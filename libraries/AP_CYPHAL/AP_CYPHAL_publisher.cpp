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

#include <AP_CYPHAL/AP_CYPHAL_publisher.h>

#if HAL_ENABLE_CYPHAL_DRIVERS

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_CYPHAL/AP_CYPHAL_registers.h>


void CyphalPublisherManager::init(CanardInstance &ins,
                                  CanardTxQueue& tx_queue,
                                  const CyphalSubscriberManager& sub_manager)
{
    CyphalBasePublisher *publisher;

    ///< Minimal requrement
    publisher = new CyphalHeartbeatPublisher(ins, tx_queue);
    add_publisher(publisher);

    publisher = new CyphalPortListPublisher(ins, tx_queue, *this, sub_manager);
    add_publisher(publisher);
}

bool CyphalPublisherManager::add_publisher(CyphalBasePublisher *publisher)
{
    if (publisher == nullptr || number_of_publishers >= max_number_of_publishers) {
        return false;
    }

    publishers[number_of_publishers] = publisher;
    number_of_publishers++;
    return true;
}

void CyphalPublisherManager::process_all()
{
    for (uint_fast8_t pub_idx = 0; pub_idx < number_of_publishers; pub_idx++) {
        if (publishers[pub_idx] != nullptr && publishers[pub_idx]->get_port_id() != 0) {
            publishers[pub_idx]->update();
        }
    }
}

void CyphalPublisherManager::fill_publishers(uavcan_node_port_SubjectIDList_0_1& publishers_list) const
{
    uavcan_node_port_SubjectIDList_0_1_select_sparse_list_(&publishers_list);
    int_fast8_t enabled_pub_amount = 0;
    for (uint_fast8_t pub_idx = 0; pub_idx < number_of_publishers; pub_idx++) {
        if (publishers[pub_idx]->is_enabled()) {
            publishers_list.sparse_list.elements[enabled_pub_amount].value = publishers[pub_idx]->get_port_id();
            enabled_pub_amount++;
        }
    }
    publishers_list.sparse_list.count = enabled_pub_amount;
}


void CyphalBasePublisher::push(size_t buf_size, uint8_t* buf)
{
    const CanardMicrosecond tx_deadline_usec = AP_HAL::micros() + 50000;
    auto result = canardTxPush(&_tx_queue, &_canard, tx_deadline_usec, &_transfer_metadata, buf_size, buf);
    static uint32_t prev_gcs_send_time_ms{1000};
    static long unsigned int counter{0};

    if (result < 0) {
        counter++;
    }

    uint32_t crnt_time_ms = AP_HAL::millis();
    if (result < 0 && prev_gcs_send_time_ms + 1000 < crnt_time_ms) {
        prev_gcs_send_time_ms = crnt_time_ms;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AP_CYPHAL: not enough mem: %ld %lu", result, counter);
    }
}


/**
 * @note uavcan.node.Heartbeat_1_0
 */
CyphalHeartbeatPublisher::CyphalHeartbeatPublisher(CanardInstance &ins, CanardTxQueue& tx_queue) :
    CyphalBasePublisher(ins, tx_queue, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_)
{
    msg.health.value = uavcan_node_Health_1_0_NOMINAL;
    msg.mode.value = uavcan_node_Mode_1_0_OPERATIONAL;
    msg.vendor_specific_status_code = static_cast<uint8_t>(0);

    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    _transfer_metadata.transfer_id = 0;
}

void CyphalHeartbeatPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void CyphalHeartbeatPublisher::publish()
{
    msg.uptime = AP_HAL::millis() / 1000;

    uint8_t buf[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    int32_t result = uavcan_node_Heartbeat_1_0_serialize_(&msg, buf, &buf_size);
    if (NUNAVUT_SUCCESS == result) {
        push(buf_size, buf);
    }

    _transfer_metadata.transfer_id++;
}


/**
 * @note uavcan.node.port.List_0_1
 */
CyphalPortListPublisher::CyphalPortListPublisher(CanardInstance &ins,
                                                 CanardTxQueue& tx_queue,
                                                 const CyphalPublisherManager& pub_manager,
                                                 const CyphalSubscriberManager& sub_manager) :
    CyphalBasePublisher(ins, tx_queue, uavcan_node_port_List_0_1_FIXED_PORT_ID_),
    _pub_manager(pub_manager),
    _sub_manager(sub_manager)
{
    _transfer_metadata.priority = CanardPriorityNominal;
    _transfer_metadata.transfer_kind = CanardTransferKindMessage;
    _transfer_metadata.port_id = uavcan_node_port_List_0_1_FIXED_PORT_ID_;
    _transfer_metadata.remote_node_id = CANARD_NODE_ID_UNSET;
    _transfer_metadata.transfer_id = 0;
}

void CyphalPortListPublisher::update()
{
    if (AP_HAL::millis() < next_publish_time_ms) {
        return;
    }
    next_publish_time_ms += publish_period_ms;

    publish();
}

void CyphalPortListPublisher::publish()
{
    static uavcan_node_port_List_0_1 msg{};

    _pub_manager.fill_publishers(msg.publishers);
    _sub_manager.fill_subscribers(msg.subscribers);

    static uint8_t buf[uavcan_node_port_List_0_1_EXTENT_BYTES_];
    size_t buf_size = uavcan_node_port_List_0_1_EXTENT_BYTES_;
    auto result = uavcan_node_port_List_0_1_serialize_(&msg, buf, &buf_size);

    if (NUNAVUT_SUCCESS == result) {
        push(buf_size, buf);
    }

    _transfer_metadata.transfer_id++;
}

#endif // HAL_ENABLE_CYPHAL_DRIVERS
