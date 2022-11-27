#include "AP_RangeFinder_CYPHAL.h"

#if HAL_ENABLE_CYPHAL_DRIVERS

#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

extern const AP_HAL::HAL& hal;

void CyphalRangefinderSubscriber::subscribe()
{
    subscribeOnMessage(uavcan_si_sample_length_Scalar_1_0_EXTENT_BYTES_);
}

void CyphalRangefinderSubscriber::handler(const CanardRxTransfer* transfer)
{
    if (_driver == nullptr) {
        return;
    }

    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_si_sample_length_Scalar_1_0 msg;
    if (uavcan_si_sample_length_Scalar_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    _driver->_distance_cm = msg.meter * 100.0f;
    _driver->_sensor_type = MAV_DISTANCE_SENSOR_LASER;
    _driver->_last_reading_ms = AP_HAL::millis();
    _driver->_status = RangeFinder::Status::Good;
    _driver->new_data = true;
}


AP_RangeFinder_CYPHAL::AP_RangeFinder_CYPHAL(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params), _rangefinder_sub(this)
{
    if (!AP_CYPHAL::add_subscriber(0, &_rangefinder_sub)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_RangeFinder_CYPHAL add_subscriber fail");
    }
}

void AP_RangeFinder_CYPHAL::update()
{
    WITH_SEMAPHORE(_sem);
    if ((AP_HAL::millis() - _last_reading_ms) > 500) {
        set_status(RangeFinder::Status::NoData);
    } else if (_status == RangeFinder::Status::Good && new_data) {
        state.distance_m = _distance_cm * 0.01f;
        state.last_reading_ms = _last_reading_ms;
        update_status();
        new_data = false;
    } else if (_status != RangeFinder::Status::Good) {
        set_status(_status);
    }
}

#endif  // HAL_ENABLE_CYPHAL_DRIVERS
