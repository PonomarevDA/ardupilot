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
#include "AP_GPS_CYPHAL.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_CYPHAL/AP_CYPHAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/definitions.h>
#include "reg/udral/physics/kinematics/geodetic/PointStateVarTs_0_1.h"
#include "uavcan/si/sample/angle/Scalar_1_0.h"
#include "uavcan/primitive/scalar/Integer16_1_0.h"

extern const AP_HAL::HAL& hal;

AP_GPS_CYPHAL::DetectedModules AP_GPS_CYPHAL::_detected_modules[] = {0};
HAL_Semaphore AP_GPS_CYPHAL::_sem_registry;


void CyphalGpsPointSubscriber::subscribe()
{
    if (_driver == nullptr) {
        return;
    }
    subscribeOnMessage(reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1_EXTENT_BYTES_);
}
void CyphalGpsPointSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1 msg;
    if (reg_udral_physics_kinematics_geodetic_PointStateVarTs_0_1_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }
    Location loc = {};
    loc.lat = msg.value.position.value.latitude * (10000000 * 180 / M_PI);
    loc.lng = msg.value.position.value.longitude * (10000000 * 180 / M_PI);
    loc.alt = msg.value.position.value.altitude.meter * 100;
    _driver->state.location = loc;

    _driver->state.velocity.x = msg.value.velocity.value.meter_per_second[0];
    _driver->state.velocity.y = msg.value.velocity.value.meter_per_second[1];
    _driver->state.velocity.z = msg.value.velocity.value.meter_per_second[2];

    _driver->handle();
}


void CyphalGpsYawSubscriber::subscribe()
{
    if (_driver == nullptr) {
        return;
    }
    subscribeOnMessage(uavcan_si_sample_angle_Scalar_1_0_EXTENT_BYTES_);
}
void CyphalGpsYawSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_si_sample_angle_Scalar_1_0 msg;
    if (uavcan_si_sample_angle_Scalar_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }
}


void CyphalGpsSatellitesSubscriber::subscribe()
{
    if (_driver == nullptr) {
        return;
    }
    subscribeOnMessage(uavcan_primitive_scalar_Integer16_1_0_EXTENT_BYTES_);
}
void CyphalGpsSatellitesSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_primitive_scalar_Integer16_1_0 msg;
    if (uavcan_primitive_scalar_Integer16_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }
    _driver->state.num_sats = msg.value;
}


void CyphalGpsStatusSubscriber::subscribe()
{
    if (_driver == nullptr) {
        return;
    }
    subscribeOnMessage(uavcan_primitive_scalar_Integer16_1_0_EXTENT_BYTES_);
}
void CyphalGpsStatusSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_primitive_scalar_Integer16_1_0 msg;
    if (uavcan_primitive_scalar_Integer16_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    AP_GPS::GPS_Status gps_status;
    switch(msg.value) {
    case 1:
        gps_status = AP_GPS::NO_FIX;
        break;
    case 2:
        gps_status = AP_GPS::GPS_OK_FIX_2D;
        break;
    case 3:
        gps_status = AP_GPS::GPS_OK_FIX_3D;
        break;
    default:
        gps_status = AP_GPS::NO_GPS;
        break;
    }
    _driver->state.status = gps_status;
}


void CyphalGpsPdopSubscriber::subscribe()
{
    if (_driver == nullptr) {
        return;
    }
    subscribeOnMessage(uavcan_primitive_scalar_Integer16_1_0_EXTENT_BYTES_);
}
void CyphalGpsPdopSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_primitive_scalar_Integer16_1_0 msg;
    if (uavcan_primitive_scalar_Integer16_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }
    _driver->state.vdop = msg.value * 100.0;
    _driver->state.hdop = msg.value * 100.0;
}


AP_GPS_CYPHAL::AP_GPS_CYPHAL(AP_GPS &_gps, AP_GPS::GPS_State &_state) :
    AP_GPS_Backend(_gps, _state, nullptr),
    _gps_point_sub(this),
    _gps_yaw_sub(this),
    _gps_sats_sub(this),
    _gps_status_sub(this),
    _gps_pdop_sub(this)
{
}

AP_GPS_Backend* AP_GPS_CYPHAL::probe(AP_GPS &_gps, AP_GPS::GPS_State &_state)
{
    AP_GPS_CYPHAL* backend;

    uint_fast8_t module_idx = 0;
    if (_detected_modules[module_idx].driver != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP_GPS_CYPHAL driver supports only a single instance yet.");
        return nullptr;
    }

    backend = new AP_GPS_CYPHAL(_gps, _state);
    if (backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_GPS_CYPHAL new returns nullptr");
        return nullptr;
    }

    if (!backend->is_port_id_correct()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AP_GPS_CYPHAL is selected, but port id are not configured. Fix and reboot.");
    } else if (!backend->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_GPS_CYPHAL Can't initialize subscriber.");
        delete backend;
        return nullptr;
    }

    _detected_modules[module_idx].driver = backend;
    return backend;
}

bool AP_GPS_CYPHAL::is_port_id_correct()
{
    if (_gps_point_sub.get_port_id() == 0 || _gps_yaw_sub.get_port_id() == 0) {
        return false;
    }
    return true;
}

bool AP_GPS_CYPHAL::init()
{
    if (!AP_CYPHAL::add_subscriber(0, &_gps_point_sub) ||
        !AP_CYPHAL::add_subscriber(0, &_gps_yaw_sub)) {
        return false;
    }

    if (_gps_sats_sub.get_port_id() != 0 && !AP_CYPHAL::add_subscriber(0, &_gps_sats_sub)) {
        return false;
    }

    if (_gps_status_sub.get_port_id() != 0 && !AP_CYPHAL::add_subscriber(0, &_gps_status_sub)) {
        return false;
    }

    if (_gps_pdop_sub.get_port_id() != 0 && !AP_CYPHAL::add_subscriber(0, &_gps_pdop_sub)) {
        return false;
    }

    return true;
}

void AP_GPS_CYPHAL::handle()
{
    state.time_week = 0;
    state.time_week_ms = 0;

    state.have_vertical_velocity = true;

    state.ground_course = wrap_360(degrees(atan2f(state.velocity.y, state.velocity.x)));
    state.ground_speed = state.velocity.xy().length();

    state.have_speed_accuracy = true;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;
    state.have_vertical_velocity = true;

    state.horizontal_accuracy = 0.1;
    state.vertical_accuracy = 0.1;
    state.speed_accuracy = 0.1;

    state.last_gps_time_ms = AP_HAL::millis();
}

bool AP_GPS_CYPHAL::read(void)
{
    return true;
}

#endif // HAL_ENABLE_CYPHAL_DRIVERS
