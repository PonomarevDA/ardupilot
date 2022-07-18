#pragma once

#if HAL_ENABLE_CYPHAL_DRIVERS

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"
#include <AP_CYPHAL/AP_CYPHAL.h>
#include <AP_CYPHAL/AP_CYPHAL_registers.h>
#include "uavcan/si/sample/magnetic_field_strength/Vector3_1_0.h"

class AP_Compass_CYPHAL;

class CyphalCompassSubscriber : public CyphalBaseSubscriber
{
public:
    CyphalCompassSubscriber(AP_Compass_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_MAG_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_Compass_CYPHAL* _driver;
};


class AP_Compass_CYPHAL : public AP_Compass_Backend
{
public:
    AP_Compass_CYPHAL(AP_CYPHAL* ap_cyphal, uint8_t sensor_id, uint32_t devid);
    void read(void) override;
    static AP_Compass_Backend* probe(uint8_t index);
    static uint32_t get_detected_devid(uint8_t index) { return _detected_modules[index].devid; }

    friend CyphalCompassSubscriber;
private:
    bool is_port_id_correct();
    bool init();

    uint8_t _instance;

    AP_CYPHAL* _ap_cyphal;
    uint8_t _sensor_id;
    uint32_t _devid;

    CyphalCompassSubscriber _compass_sub;

    // Module Detection Registry
    static struct DetectedModules {
        AP_CYPHAL* ap_cyphal;
        uint8_t sensor_id;
        AP_Compass_CYPHAL *driver;
        uint32_t devid;
    } _detected_modules[COMPASS_MAX_BACKEND];

    static HAL_Semaphore _sem_registry;
};

#endif