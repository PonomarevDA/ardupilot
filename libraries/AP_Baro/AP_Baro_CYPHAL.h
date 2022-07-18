#pragma once

#if HAL_ENABLE_CYPHAL_DRIVERS

#include "AP_Baro_Backend.h"
#include <AP_CYPHAL/AP_CYPHAL.h>


class AP_Baro_CYPHAL;


class CyphalBaroTemperatureSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalBaroTemperatureSubscriber(AP_Baro_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_BARO_TEMPERATURE_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_Baro_CYPHAL* _driver;
};

class CyphalBaroPressureSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalBaroPressureSubscriber(AP_Baro_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_BARO_PRESSURE_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_Baro_CYPHAL* _driver;
    static void _update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count);
};


class AP_Baro_CYPHAL : public AP_Baro_Backend {
public:
    AP_Baro_CYPHAL(AP_Baro &baro);
    void update() override;
    static AP_Baro_Backend* probe(AP_Baro &baro);
    bool is_port_id_correct();
    bool init();

    friend CyphalBaroTemperatureSubscriber;
    friend CyphalBaroPressureSubscriber;
private:

    AP_CYPHAL* _ap_cyphal;

    float _pressure;
    float _temperature;
    bool new_pressure;
    uint8_t  _pressure_count;
    uint8_t _instance;
    uint8_t _node_id;

    HAL_Semaphore _sem_baro;

    CyphalBaroTemperatureSubscriber _baro_temperature_sub;
    CyphalBaroPressureSubscriber _baro_pressure_sub;

    // Module Detection Registry
    static constexpr uint8_t CYPHAL_BARO_MAX_SUPPORTED_DRIVERS = 1;
    static struct DetectedModules {
        AP_CYPHAL* ap_cyphal;
        uint8_t node_id;
        AP_Baro_CYPHAL* driver;
    } _detected_modules[CYPHAL_BARO_MAX_SUPPORTED_DRIVERS];
};

#endif // HAL_ENABLE_CYPHAL_DRIVERS