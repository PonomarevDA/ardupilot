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
#include "AP_InertialSensor_DRONECAN.h"

#if AP_INS_DRONECAN_ENABLED

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;
uint8_t AP_InertialSensor_DRONECAN::instances_amount = 0;
AP_InertialSensor_DRONECAN::DetectedModules AP_InertialSensor_DRONECAN::_detected_modules[] = {0};
static uint32_t imu_ts_ms = 0;
static uint32_t max_timeout_ms = 0;
static size_t msg_counter{0};

void AP_InertialSensor_DRONECAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    if (ap_dronecan == nullptr) {
        return;
    }

    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_raw_imu, ap_dronecan->get_driver_index()) == nullptr) {
        AP_BoardConfig::allocation_error("imu_sub");
    }
}

AP_InertialSensor_DRONECAN::AP_InertialSensor_DRONECAN(AP_InertialSensor &imu)
    : AP_InertialSensor_Backend(imu)
{
    _detected_modules[0].driver = this;
    instances_amount++;
}

bool AP_InertialSensor_DRONECAN::init()
{
    return true;
}

void AP_InertialSensor_DRONECAN::start()
{
    static uint8_t bus_id = 0;
    uint32_t gyro_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN, bus_id, 1, DEVTYPE_INS_UAVCAN);   // 3997955
    uint32_t accel_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN, bus_id, 2, DEVTYPE_INS_UAVCAN);  // 3998211
    bus_id++;
    const float rate = 300;
    if (_imu.register_gyro(_detected_modules[0].gyro_instance, rate, gyro_id) && _imu.register_accel(_detected_modules[0].accel_instance, rate, accel_id)) {
        started = true;
    }
    set_gyro_orientation(_detected_modules[0].gyro_instance, ROTATION_NONE);
    set_accel_orientation(_detected_modules[0].accel_instance, ROTATION_NONE);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                  "Use DroneCAN HITL IMU: gyro=%lu accel=%lu.",
                  (long unsigned int)gyro_id,
                  (long unsigned int)accel_id
    );
    if (_imu.get_accel_count() != 1 || !_imu.accel_calibrated_ok_all()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                    "Wrong HITL Accel setup: num=%lu calib_res=%i.",
                    (long unsigned int)_imu.get_accel_count(),
                    (int8_t)_imu.accel_calibrated_ok_all()
        );
    }
    if (_imu.get_accel_count() != 1 || !_imu.gyro_calibrated_ok_all()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,
                    "Wrong HITL Gyro setup: num=%lu calib_res=%i.",
                    (long unsigned int)_imu.get_gyro_count(),
                    (int8_t)_imu.gyro_calibrated_ok_all()
        );
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_DRONECAN::loop, void),
                                      "UC_IMU",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("AP_InertialSensor_DRONECAN: Failed to create thread");
    }
}

void AP_InertialSensor_DRONECAN::loop()
{
    while (true) {
        hal.scheduler->delay_microseconds(5000);
        if (imu_ts_ms == 0 || imu_ts_ms + 100 < AP_HAL::millis()) {
            Vector3f accel{0.00f, 0.00f, -9.81f};
            Vector3f gyro{0.00f, 0.00f, 0.00f};
            publish_accel(accel);
            publish_gyro(gyro);
        }

        static uint32_t logger_next_time_ms = 10000;
        if (AP_HAL::millis() > logger_next_time_ms) {
            logger_next_time_ms = AP_HAL::millis() + 10000;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                        "Hitl IMU: %lu msg/sec, max timeout=%lu ms.",
                        (long unsigned int)(msg_counter / 10),
                        (long unsigned int)(max_timeout_ms)
            );
            msg_counter = 0;
            max_timeout_ms = 0;
        }
    }
}

bool AP_InertialSensor_DRONECAN::update()
{
    if (started) {
        update_accel(_detected_modules[0].accel_instance);
        update_gyro(_detected_modules[0].gyro_instance);
    }
    return started;
}

void AP_InertialSensor_DRONECAN::publish_accel(Vector3f& accel)
{
    _rotate_and_correct_accel(_detected_modules[0].accel_instance, accel);
    _notify_new_accel_raw_sample(_detected_modules[0].accel_instance, accel);
}

void AP_InertialSensor_DRONECAN::publish_gyro(Vector3f& gyro)
{
    _rotate_and_correct_gyro(_detected_modules[0].gyro_instance, gyro);
    _notify_new_gyro_raw_sample(_detected_modules[0].gyro_instance, gyro);
}

void AP_InertialSensor_DRONECAN::handle_raw_imu(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_RawIMU &msg)
{
    if (_detected_modules[0].driver == nullptr) {
        return;
    }

    msg_counter++;
    max_timeout_ms = AP_HAL::millis() - imu_ts_ms;
    imu_ts_ms = AP_HAL::millis();

    Vector3f accel{
        msg.accelerometer_latest[0],
        msg.accelerometer_latest[1],
        msg.accelerometer_latest[2]
    };
    _detected_modules[0].driver->publish_accel(accel);

    Vector3f gyro{
        msg.rate_gyro_latest[0],
        msg.rate_gyro_latest[1],
        msg.rate_gyro_latest[2]
    };
    _detected_modules[0].driver->publish_gyro(gyro);
}

#endif
