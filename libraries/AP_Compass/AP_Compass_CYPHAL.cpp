/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include "AP_Compass_CYPHAL.h"
#include <AP_CANManager/AP_CANManager.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;
#define LOG_TAG "COMPASS"

AP_Compass_CYPHAL::DetectedModules AP_Compass_CYPHAL::_detected_modules[] = {0};
HAL_Semaphore AP_Compass_CYPHAL::_sem_registry;

void CyphalCompassSubscriber::subscribe()
{
    subscribeOnMessage(uavcan_si_sample_magnetic_field_strength_Vector3_1_0_EXTENT_BYTES_);
}
void CyphalCompassSubscriber::handler(const CanardRxTransfer* transfer)
{
    if (_driver == nullptr) {
        return;
    }

    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_si_sample_magnetic_field_strength_Vector3_1_0 msg;
    if (uavcan_si_sample_magnetic_field_strength_Vector3_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    Vector3f mag_vector;
    mag_vector[0] = msg.tesla[0] * 10000000;
    mag_vector[1] = msg.tesla[1] * 10000000;
    mag_vector[2] = msg.tesla[2] * 10000000;
    _driver->accumulate_sample(mag_vector, 0);
}

AP_Compass_CYPHAL::AP_Compass_CYPHAL(AP_CYPHAL* ap_cyphal, uint8_t sensor_id, uint32_t devid)
    : _ap_cyphal(ap_cyphal)
    , _sensor_id(sensor_id)
    , _devid(devid)
    , _compass_sub(this)
{
}

AP_Compass_Backend* AP_Compass_CYPHAL::probe(uint8_t index)
{
    ///< support only a single sensor
    if (index != 0) {
        return nullptr;
    }

    _detected_modules[index].ap_cyphal = AP_CYPHAL::get_cyphal(0);
    if (_detected_modules[index].ap_cyphal == nullptr) {
        return nullptr;
    }

    uint32_t devid = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN, 0, 42, index + 1);
    _detected_modules[index].devid = devid;
    _detected_modules[index].sensor_id = index;

    ///< Try to register new Compass mode to a backend
    AP_Compass_CYPHAL* driver = new AP_Compass_CYPHAL(_detected_modules[index].ap_cyphal,
                                                      _detected_modules[index].sensor_id,
                                                      _detected_modules[index].devid);
    if (driver == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Compass_CYPHAL new operation fails");
        return nullptr;
    } else if (!driver->is_port_id_correct()) {
        delete driver;
        return nullptr;
    } else if (!driver->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Compass_CYPHAL init fail");
        delete driver;
        return nullptr;
    }

    _detected_modules[index].driver = driver;
    return driver;
}

bool AP_Compass_CYPHAL::is_port_id_correct()
{
    if (_compass_sub.get_port_id() == 0) {
        return false;
    }
    return true;
}

bool AP_Compass_CYPHAL::init()
{
    // Adding 1 is necessary to allow backward compatibilty, where this field was set as 1 by default
    if (!register_compass(_devid, _instance)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Compass_CYPHAL register_compass fail");
        return false;
    }

    set_dev_id(_instance, _devid);
    set_external(_instance, true);

    if (!AP_CYPHAL::add_subscriber(0, &_compass_sub)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Compass_CYPHAL add_subscriber fail");
        return false;
    }
    return true;
}

void AP_Compass_CYPHAL::read(void)
{
    drain_accumulated_samples(_instance);
}

#endif