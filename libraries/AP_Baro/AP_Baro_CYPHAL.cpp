#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include "AP_Baro_CYPHAL.h"
#include <AP_CANManager/AP_CANManager.h>
#include <AP_CYPHAL/AP_CYPHAL.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "uavcan/si/sample/temperature/Scalar_1_0.h"
#include "uavcan/si/sample/pressure/Scalar_1_0.h"

extern const AP_HAL::HAL& hal;
AP_Baro_CYPHAL::DetectedModules AP_Baro_CYPHAL::_detected_modules[] = {0};

void CyphalBaroTemperatureSubscriber::subscribe()
{
    if (_driver == nullptr) {
        return;
    }
    subscribeOnMessage(uavcan_si_sample_temperature_Scalar_1_0_EXTENT_BYTES_);
}

void CyphalBaroTemperatureSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_si_sample_temperature_Scalar_1_0 msg;
    if (uavcan_si_sample_temperature_Scalar_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }
    {
        WITH_SEMAPHORE(_driver->_sem_baro);
        _driver->_temperature = KELVIN_TO_C(msg.kelvin);
    }
}

void CyphalBaroPressureSubscriber::subscribe()
{
    if (_driver == nullptr) {
        return;
    }
    subscribeOnMessage(uavcan_si_sample_pressure_Scalar_1_0_EXTENT_BYTES_);
}

void CyphalBaroPressureSubscriber::handler(const CanardRxTransfer* transfer)
{
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    size_t payload_len = transfer->payload_size;
    uavcan_si_sample_pressure_Scalar_1_0 msg;
    if (uavcan_si_sample_pressure_Scalar_1_0_deserialize_(&msg, payload, &payload_len) < 0) {
        return;
    }

    {
        WITH_SEMAPHORE(_driver->_sem_baro);
        _update_and_wrap_accumulator(&_driver->_pressure, msg.pascal, &_driver->_pressure_count, 32);
        _driver->new_pressure = true;
    }
}

void CyphalBaroPressureSubscriber::_update_and_wrap_accumulator(float *accum,
                                                                float val,
                                                                uint8_t *count,
                                                                const uint8_t max_count)
{
    *accum += val;
    *count += 1;
    if (*count == max_count) {
        *count = max_count / 2;
        *accum = *accum / 2;
    }
}

AP_Baro_CYPHAL::AP_Baro_CYPHAL(AP_Baro &baro) :
    AP_Baro_Backend(baro), _baro_temperature_sub(this), _baro_pressure_sub(this)
{}

AP_Baro_Backend* AP_Baro_CYPHAL::probe(AP_Baro &baro)
{
    // suport only a single barometer yet
    const uint_fast8_t module_idx = 0;
    if (_detected_modules[module_idx].driver != nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP_Baro_CYPHAL supports only one instance yet.");
        return nullptr;
    }

    _detected_modules[module_idx].ap_cyphal = AP_CYPHAL::get_cyphal(0);
    if (_detected_modules[module_idx].ap_cyphal == nullptr) {
        return nullptr;
    }

    AP_Baro_CYPHAL* backend = new AP_Baro_CYPHAL(baro);
    if (backend == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Baro_CYPHAL: new operation returns nullptr.");
        return nullptr;
    }

    if (!backend->is_port_id_correct()) {
        delete backend;
        return nullptr;
    } else if (!backend->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Baro_CYPHAL init fail: not enough mem for subs.");
        delete backend;
        return nullptr;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP_Baro_CYPHAL init ok");
    }

    _detected_modules[module_idx].driver = backend;
    backend->_pressure = 0;
    backend->_pressure_count = 0;
    backend->_ap_cyphal = _detected_modules[module_idx].ap_cyphal;
    backend->_node_id = _detected_modules[module_idx].node_id;
    backend->_instance = backend->_frontend.register_sensor();
    backend->set_bus_id(backend->_instance,
                        AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                    _detected_modules[module_idx].ap_cyphal->get_driver_index(),
                                                    backend->_node_id,
                                                    0));
    return backend;
}

bool AP_Baro_CYPHAL::is_port_id_correct()
{
    if (_baro_temperature_sub.get_port_id() == 0 ||
        _baro_pressure_sub.get_port_id() == 0) {
        return false;
    }
    return true;
}

bool AP_Baro_CYPHAL::init()
{
    if (!AP_CYPHAL::add_subscriber(0, &_baro_temperature_sub) ||
        !AP_CYPHAL::add_subscriber(0, &_baro_pressure_sub)) {
        return false;
    }
    return true;
}

void AP_Baro_CYPHAL::update(void)
{
    float pressure = 0;
    WITH_SEMAPHORE(_sem_baro);
    if (new_pressure) {
        if (_pressure_count != 0) {
            pressure = _pressure / _pressure_count;
            _pressure_count = 0;
            _pressure = 0;
        }
        _copy_to_frontend(_instance, pressure, _temperature);

        _frontend.set_external_temperature(_temperature);
        new_pressure = false;
    }
}

#endif // HAL_ENABLE_CYPHAL_DRIVERS
