#pragma once
#include "AP_InertialSensor_config.h"

#if AP_INS_DRONECAN_ENABLED
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_InertialSensor_DRONECAN : public AP_InertialSensor_Backend {
public:
    AP_InertialSensor_DRONECAN(AP_InertialSensor &imu);
    bool update() override;
    void start() override;
    void publish_accel(Vector3f& accel);
    void publish_gyro(Vector3f& gyro);
    static uint8_t instances_amount;

    static void subscribe_msgs(AP_DroneCAN* ap_dronecan);
    static void handle_raw_imu(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_RawIMU &msg);

private:
    bool init();
    void loop(void);
    bool started;
    // Module Detection Registry
    static struct DetectedModules {
        AP_InertialSensor_DRONECAN *driver;
        uint8_t gyro_instance;
        uint8_t accel_instance;
    } _detected_modules[1];
};
#endif
