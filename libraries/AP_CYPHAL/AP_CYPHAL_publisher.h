/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Dmitry Ponomarev
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include "canard.h"

#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/port/List_0_1.h"
#include "uavcan/node/GetInfo_1_0.h"

#include <AP_CYPHAL/AP_CYPHAL_subscriber.h>

class CyphalBasePublisher;


class CyphalPublisherManager
{
public:
    CyphalPublisherManager() {};
    void init(CanardInstance &ins, CanardTxQueue& tx_queue, const CyphalSubscriberManager& sub_manager);

    // return true in sucess, otherwise false
    bool add_publisher(CyphalBasePublisher *publisher);

    void fill_publishers(uavcan_node_port_SubjectIDList_0_1& publishers) const;

    void process_all();
private:
    static constexpr uint8_t max_number_of_publishers = 4;
    uint8_t number_of_publishers = 0;
    CyphalBasePublisher *publishers[max_number_of_publishers];
};


class CyphalBasePublisher
{
public:
    CyphalBasePublisher(CanardInstance &ins, CanardTxQueue& tx_queue, CanardPortID port_id) :
        _canard(ins), _tx_queue(tx_queue), _port_id(port_id) {};
    uint16_t get_port_id()
    {
        return _port_id;
    }
    bool is_enabled()
    {
        return (_port_id == 0 || _port_id > 8191) ? false : true;
    }
    virtual void update() = 0;

protected:
    void push(size_t buf_size, uint8_t* buf);

    CanardInstance &_canard;
    CanardTxQueue &_tx_queue;
    CanardPortID _port_id;
    CanardTransferMetadata _transfer_metadata;
};


/**
 * @note uavcan.node.Heartbeat_1_0
 */
class CyphalHeartbeatPublisher : public CyphalBasePublisher
{
public:
    CyphalHeartbeatPublisher(CanardInstance &ins, CanardTxQueue& tx_queue);
    virtual void update() override;

private:
    static constexpr uint32_t publish_period_ms = 500;
    uint32_t next_publish_time_ms{publish_period_ms};
    uavcan_node_Heartbeat_1_0 msg;

    void publish();
};


/**
 * @note uavcan.node.port.List_0_1
 */
class CyphalPortListPublisher : public CyphalBasePublisher
{
public:
    CyphalPortListPublisher(CanardInstance &ins,
                            CanardTxQueue& tx_queue,
                            const CyphalPublisherManager& pub_manager,
                            const CyphalSubscriberManager& sub_manager);
    virtual void update() override;

private:
    static constexpr uint32_t publish_period_ms = 3000;
    uint32_t next_publish_time_ms{publish_period_ms};
    const CyphalPublisherManager& _pub_manager;
    const CyphalSubscriberManager& _sub_manager;
    // uavcan_node_port_List_0_1 msg;   ///< @todo this data type requires too much memory

    void publish();
};

#endif // HAL_ENABLE_CYPHAL_DRIVERS
