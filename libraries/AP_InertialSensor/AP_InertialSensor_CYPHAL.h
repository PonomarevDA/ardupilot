#pragma once

#if HAL_ENABLE_CYPHAL_DRIVERS

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_CYPHAL/AP_CYPHAL.h>
#include "uavcan/si/sample/angular_velocity/Vector3_1_0.h"
#include "uavcan/si/sample/acceleration/Vector3_1_0.h"
#include <AP_CYPHAL/AP_CYPHAL_publisher.h>
#include <AP_CYPHAL/AP_CYPHAL_subscriber.h>
#include <AP_CYPHAL/AP_CYPHAL_registers.h>


class AP_InertialSensor_CYPHAL;


class CyphalAccelerationSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalAccelerationSubscriber(AP_InertialSensor_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_ACCEL_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_InertialSensor_CYPHAL* _driver;
};


class CyphalGyroscopeSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalGyroscopeSubscriber(AP_InertialSensor_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_GYRO_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_InertialSensor_CYPHAL* _driver;
};

class AP_InertialSensor_CYPHAL : public AP_InertialSensor_Backend {
public:
    AP_InertialSensor_CYPHAL(AP_InertialSensor &imu);

    bool update() override;
    void start() override;

    void publish_accel(Vector3f& accel);
    void publish_gyro(Vector3f& gyro);

    static uint8_t instances_amount;
private:
    bool init();
    void loop(void);

    bool started;

    CyphalAccelerationSubscriber* _accel_sub;
    CyphalGyroscopeSubscriber* _gyro_sub;

    // Module Detection Registry
    static struct DetectedModules {
        AP_InertialSensor_CYPHAL *driver;
        uint8_t gyro_instance;
        uint8_t accel_instance;
    } _detected_modules[1];
};

#endif