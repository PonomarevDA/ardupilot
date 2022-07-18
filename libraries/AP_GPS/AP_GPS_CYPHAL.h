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

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_GPS.h"
#include "GPS_Backend.h"
#include "RTCM3_Parser.h"
#include <AP_CYPHAL/AP_CYPHAL.h>


class AP_GPS_CYPHAL;


class CyphalGpsPointSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalGpsPointSubscriber(AP_GPS_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_GPS_POINT_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_GPS_CYPHAL* _driver;
    static void _update_and_wrap_accumulator(float *accum, float val, uint8_t *count, const uint8_t max_count);
};

class CyphalGpsYawSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalGpsYawSubscriber(AP_GPS_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_GPS_YAW_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_GPS_CYPHAL* _driver;
};

class CyphalGpsSatellitesSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalGpsSatellitesSubscriber(AP_GPS_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_GPS_SATS_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_GPS_CYPHAL* _driver;
};

class CyphalGpsStatusSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalGpsStatusSubscriber(AP_GPS_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_GPS_STATUS_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_GPS_CYPHAL* _driver;
};

class CyphalGpsPdopSubscriber: public CyphalBaseSubscriber
{
public:
    CyphalGpsPdopSubscriber(AP_GPS_CYPHAL* driver) :
        CyphalBaseSubscriber(UAVCAN_SUB_GPS_PDOP_ID), _driver(driver) { };

    virtual void subscribe() override;
    virtual void handler(const CanardRxTransfer* transfer) override;
private:
    AP_GPS_CYPHAL* _driver;
};

class AP_GPS_CYPHAL : public AP_GPS_Backend {
public:
    AP_GPS_CYPHAL(AP_GPS &_gps, AP_GPS::GPS_State &_state);
    static AP_GPS_Backend* probe(AP_GPS &_gps, AP_GPS::GPS_State &_state);
    bool read() override;
    void handle();
    const char *name() const override { return "CyphalGps"; }
    bool is_port_id_correct();
    bool init();

    friend CyphalGpsPointSubscriber;
    friend CyphalGpsYawSubscriber;
    friend CyphalGpsSatellitesSubscriber;
    friend CyphalGpsStatusSubscriber;
    friend CyphalGpsPdopSubscriber;
private:
    static struct DetectedModules {
        AP_CYPHAL* ap_cyphal;
        uint8_t node_id;
        uint8_t instance;
        AP_GPS_CYPHAL* driver;
    } _detected_modules[GPS_MAX_RECEIVERS];

    CyphalGpsPointSubscriber _gps_point_sub;
    CyphalGpsYawSubscriber _gps_yaw_sub;
    CyphalGpsSatellitesSubscriber _gps_sats_sub;
    CyphalGpsStatusSubscriber _gps_status_sub;
    CyphalGpsPdopSubscriber _gps_pdop_sub;

    static HAL_Semaphore _sem_registry;
};
