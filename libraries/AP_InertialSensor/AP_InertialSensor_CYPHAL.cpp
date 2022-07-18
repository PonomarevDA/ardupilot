/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include "AP_InertialSensor_CYPHAL.h"

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;

uint8_t AP_InertialSensor_CYPHAL::instances_amount = 0;
AP_InertialSensor_CYPHAL::DetectedModules AP_InertialSensor_CYPHAL::_detected_modules[] = {0};
static HAL_Semaphore sem_registry;

static uint32_t imu_ts_ms = 0;


void CyphalAccelerationSubscriber::subscribe()
{
    subscribeOnMessage(uavcan_si_sample_acceleration_Vector3_1_0_EXTENT_BYTES_);
}
void CyphalAccelerationSubscriber::handler(const CanardRxTransfer* transfer)
{
    WITH_SEMAPHORE(sem_registry);
    if (_driver == nullptr) {
        return;
    }

    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_si_sample_acceleration_Vector3_1_0 msg;
    if (uavcan_si_sample_acceleration_Vector3_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    imu_ts_ms = AP_HAL::millis();

    Vector3f accel{
        msg.meter_per_second_per_second[0],
        msg.meter_per_second_per_second[1],
        msg.meter_per_second_per_second[2]
    };
    _driver->publish_accel(accel);
}


void CyphalGyroscopeSubscriber::subscribe()
{
    subscribeOnMessage(uavcan_si_sample_angular_velocity_Vector3_1_0_EXTENT_BYTES_);
}
void CyphalGyroscopeSubscriber::handler(const CanardRxTransfer* transfer)
{
    WITH_SEMAPHORE(sem_registry);
    if (_driver == nullptr) {
        return;
    }

    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_si_sample_angular_velocity_Vector3_1_0 msg;
    if (uavcan_si_sample_angular_velocity_Vector3_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    imu_ts_ms = AP_HAL::millis();

    Vector3f gyro{
        msg.radian_per_second[0],
        msg.radian_per_second[1],
        msg.radian_per_second[2]
    };
    _driver->publish_gyro(gyro);
}


AP_InertialSensor_CYPHAL::AP_InertialSensor_CYPHAL(AP_InertialSensor &imu)
    : AP_InertialSensor_Backend(imu)
{
    _detected_modules[0].driver = this;
    instances_amount++;
}


bool AP_InertialSensor_CYPHAL::init()
{
    return true;
}

void AP_InertialSensor_CYPHAL::start()
{
    static uint8_t bus_id = 0;
    uint32_t gyro_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN, bus_id, 1, DEVTYPE_UAVCAN);
    uint32_t accel_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN, bus_id, 2, DEVTYPE_UAVCAN);
    bus_id++;

    const float rate = 200;
    if (_imu.register_gyro(_detected_modules[0].gyro_instance, rate, gyro_id) && _imu.register_accel(_detected_modules[0].accel_instance, rate, accel_id)) {
        started = true;
    }

    set_gyro_orientation(_detected_modules[0].gyro_instance, ROTATION_NONE);
    set_accel_orientation(_detected_modules[0].accel_instance, ROTATION_NONE);

    _accel_sub = new CyphalAccelerationSubscriber(this);
    _gyro_sub = new CyphalGyroscopeSubscriber(this);

    if (!AP_CYPHAL::add_subscriber(0, _accel_sub) || !AP_CYPHAL::add_subscriber(0, _gyro_sub)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AP_InertialSensor_CYPHAL: port id are not configurated. Fix and reboot. Use dummy measurement for a while.");
    }

    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP_InertialSensor_CYPHAL: HITL mode initialized ok.");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_CYPHAL::loop, void),
                                      "UC_IMU",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("AP_InertialSensor_CYPHAL: Failed to create thread");
    }
}

void AP_InertialSensor_CYPHAL::loop()
{
    while (true) {
        hal.scheduler->delay_microseconds(5000);
        WITH_SEMAPHORE(sem_registry);
        if (imu_ts_ms == 0 || imu_ts_ms + 100 < AP_HAL::millis()) {

            Vector3f accel{0.00f, 0.00f, -9.81f};
            Vector3f gyro{0.00f, 0.00f, 0.00f};
            publish_accel(accel);
            publish_gyro(gyro);
        }
    }
}

bool AP_InertialSensor_CYPHAL::update()
{
    if (started) {
        update_accel(_detected_modules[0].accel_instance);
        update_gyro(_detected_modules[0].gyro_instance);
    }
    return started;
}

void AP_InertialSensor_CYPHAL::publish_accel(Vector3f& accel)
{
    _rotate_and_correct_accel(_detected_modules[0].accel_instance, accel);
    _notify_new_accel_raw_sample(_detected_modules[0].accel_instance, accel);
}

void AP_InertialSensor_CYPHAL::publish_gyro(Vector3f& gyro)
{
    _rotate_and_correct_gyro(_detected_modules[0].gyro_instance, gyro);
    _notify_new_gyro_raw_sample(_detected_modules[0].gyro_instance, gyro);
}

#endif