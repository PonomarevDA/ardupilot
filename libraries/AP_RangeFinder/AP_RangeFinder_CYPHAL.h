#pragma once

#include "AP_RangeFinder_Backend.h"

#if HAL_ENABLE_CYPHAL_DRIVERS

#include <AP_CYPHAL/AP_CYPHAL.h>
#include <AP_CYPHAL/AP_CYPHAL_registers.h>
#include "uavcan/si/sample/length/Scalar_1_0.h"

class MeasurementCb;

class AP_RangeFinder_CYPHAL;

class CyphalRangefinderSubscriber : public CyphalBaseSubscriber
{
public:
    CyphalRangefinderSubscriber(AP_RangeFinder_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_RNG_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_RangeFinder_CYPHAL* _driver;
};

class AP_RangeFinder_CYPHAL : public AP_RangeFinder_Backend {
public:
    AP_RangeFinder_CYPHAL(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    void update() override;

    static AP_RangeFinder_Backend* detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    friend CyphalRangefinderSubscriber;
protected:
    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return _sensor_type;
    }
private:
    uint8_t _instance;
    RangeFinder::Status _status;
    uint16_t _distance_cm;
    uint32_t _last_reading_ms;
    AP_CYPHAL* _ap_cyphal;
    uint8_t _node_id;
    bool new_data;
    MAV_DISTANCE_SENSOR _sensor_type;
    CyphalRangefinderSubscriber _rangefinder_sub;
};

#endif  // HAL_ENABLE_CYPHAL_DRIVERS
