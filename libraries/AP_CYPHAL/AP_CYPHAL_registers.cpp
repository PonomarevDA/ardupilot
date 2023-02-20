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

#include "AP_CYPHAL_registers.h"

#if HAL_ENABLE_CYPHAL_DRIVERS

#include <array>
#include "AP_CYPHAL.h"
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

CyphalRegisters* CyphalRegisters::instance;

struct RegisterCell {
    uint8_t register_name[35];
};

static RegisterCell registers_table[CyphalRegisters::NUMBER_OF_REGISTERS] = {
    {"uavcan.node.id"},

    {"uavcan.pub.note_response.id"},
    {"uavcan.pub.setpoint.id"},
    {"uavcan.pub.readiness.id"},

    {"uavcan.sub.esc_heartbeat.0.id"},
    {"uavcan.sub.feedback.0.id"},
    {"uavcan.sub.power.0.id"},
    {"uavcan.sub.status.0.id"},
    {"uavcan.sub.dynamics.0.id"},

    {"uavcan.sub.esc_heartbeat.1.id"},
    {"uavcan.sub.feedback.1.id"},
    {"uavcan.sub.power.1.id"},
    {"uavcan.sub.status.1.id"},
    {"uavcan.sub.dynamics.1.id"},

    {"uavcan.sub.esc_heartbeat.2.id"},
    {"uavcan.sub.feedback.2.id"},
    {"uavcan.sub.power.2.id"},
    {"uavcan.sub.status.2.id"},
    {"uavcan.sub.dynamics.2.id"},

    {"uavcan.sub.esc_heartbeat.3.id"},
    {"uavcan.sub.feedback.3.id"},
    {"uavcan.sub.power.3.id"},
    {"uavcan.sub.status.3.id"},
    {"uavcan.sub.dynamics.3.id"},

    {"uavcan.sub.gps.point.id"},
    {"uavcan.sub.gps.yaw.id"},
    {"uavcan.sub.gps.sats.id"},
    {"uavcan.sub.gps.status.id"},
    {"uavcan.sub.gps.pdop.id"},

    {"uavcan.sub.mag.id"},

    {"uavcan.sub.baro.temp.id"},
    {"uavcan.sub.baro.pres.id"},

    {"uavcan.pub.setpoint.type"},
    {"uavcan.pub.readiness.type"},

    {"uavcan.sub.esc_heartbeat.0.type"},
    {"uavcan.sub.feedback.0.type"},
    {"uavcan.sub.power.0.type"},
    {"uavcan.sub.status.0.type"},
    {"uavcan.sub.dynamics.0.type"},

    {"uavcan.sub.esc_heartbeat.1.type"},
    {"uavcan.sub.feedback.1.type"},
    {"uavcan.sub.power.1.type"},
    {"uavcan.sub.status.1.type"},
    {"uavcan.sub.dynamics.1.type"},

    {"uavcan.sub.esc_heartbeat.2.type"},
    {"uavcan.sub.feedback.2.type"},
    {"uavcan.sub.power.2.type"},
    {"uavcan.sub.status.2.type"},
    {"uavcan.sub.dynamics.2.type"},

    {"uavcan.sub.esc_heartbeat.3.type"},
    {"uavcan.sub.feedback.3.type"},
    {"uavcan.sub.power.3.type"},
    {"uavcan.sub.status.3.type"},
    {"uavcan.sub.dynamics.3.type"},

    {"uavcan.sub.gps.point.type"},
    {"uavcan.sub.gps.yaw.type"},
    {"uavcan.sub.gps.sats.type"},
    {"uavcan.sub.gps.status.type"},
    {"uavcan.sub.gps.pdop.type"},
    {"uavcan.sub.mag.type"},
    {"uavcan.sub.baro.temp.type"},
    {"uavcan.sub.baro.pres.type"},
};

static const std::array<const char*, CyphalRegisters::NUMBER_OF_STRING_REGISTERS> string_registers = {
    "reg.udral.service.actuator.common.sp.Vector4",
    "reg.udral.service.common.Readiness",

    "None",
    "reg.udral.service.actuator.common.Feedback",
    "reg.udral.physics.electricity.PowerTs",
    "reg.udral.service.actuator.common.Status",
    "reg.udral.physics.dynamics.rotation.PlanarTs",

    "None",
    "reg.udral.service.actuator.common.Feedback",
    "reg.udral.physics.electricity.PowerTs",
    "reg.udral.service.actuator.common.Status",
    "reg.udral.physics.dynamics.rotation.PlanarTs",

    "None",
    "reg.udral.service.actuator.common.Feedback",
    "reg.udral.physics.electricity.PowerTs",
    "reg.udral.service.actuator.common.Status",
    "reg.udral.physics.dynamics.rotation.PlanarTs",

    "None",
    "reg.udral.service.actuator.common.Feedback",
    "reg.udral.physics.electricity.PowerTs",
    "reg.udral.service.actuator.common.Status",
    "reg.udral.physics.dynamics.rotation.PlanarTs",

    "reg.udral.physics.kinematics.geodetic.PointStateVarTs",
    "uavcan.si.sample.angle.Scalar",
    "uavcan.primitive.scalar.Integer16",
    "uavcan.primitive.scalar.Integer16",
    "uavcan.primitive.scalar.Integer16",
    "uavcan.si.sample.magnetic_field_strength.Vector3",
    "uavcan.si.sample.temperature.Scalar",
    "uavcan.si.sample.pressure.Scalar",
};

bool CyphalRegisters::init(CyphalSubscriberManager &sub_manager, CanardInstance &ins, CanardTxQueue &tx_queue)
{
    CyphalBaseSubscriber *subsriber;

    subsriber = new CyphalRegisterListRequest(ins, tx_queue, *this);
    sub_manager.add_subscriber(subsriber);

    subsriber = new CyphalRegisterAccessRequest(ins, tx_queue, *this);
    sub_manager.add_subscriber(subsriber);

    return true;
}


int8_t CyphalRegisters::getRegisterIndexByRegisterName(const uint8_t register_name[], uint8_t name_length)
{
    if (name_length == 0) {
        return -1;
    }

    int8_t reg_idx;
    for (reg_idx = 0; reg_idx < NUMBER_OF_REGISTERS; reg_idx++) {
        bool is_equal = true;
        for (uint_fast8_t byte_idx = 0; byte_idx < name_length; byte_idx++) {
            if (registers_table[reg_idx].register_name[byte_idx] != register_name[byte_idx]) {
                is_equal = false;
                break;
            }
        }
        if (is_equal) {
            return reg_idx;
        }
    }

    return -1;
}

uint8_t CyphalRegisters::getRegisterNameByIndex(uint8_t register_index, uint8_t register_name[])
{
    if (register_index >= NUMBER_OF_REGISTERS) {
        return 0;
    }

    uint8_t len = strlen((const char*)(registers_table[register_index].register_name));
    memcpy(register_name, registers_table[register_index].register_name, len);

    return len;
}

int16_t CyphalRegisters::getPortIdByIndex(uint8_t param_idx)
{
    return (param_idx < NUMBER_OF_INTEGER_REGISTERS) ? _parameters_table[param_idx].get() : -1;
}

void CyphalRegisters::setPortIdByIndex(uint8_t param_idx, int16_t new_port_id)
{
    if (param_idx >= NUMBER_OF_INTEGER_REGISTERS) {
        return;
    }

    _parameters_table[param_idx].set_and_save(new_port_id);
}

/**
 * @note uavcan.register.Access.1.0
 */
void CyphalRegisterAccessRequest::subscribe()
{
    subscribeOnRequest(uavcan_register_Access_Request_1_0_EXTENT_BYTES_);
}

void CyphalRegisterAccessRequest::handler(const CanardRxTransfer* transfer)
{
    auto reg_index = parseRequest(transfer);
    makeResponse(transfer, reg_index);
}

int8_t CyphalRegisterAccessRequest::parseRequest(const CanardRxTransfer* transfer)
{
    auto payload_len = transfer->payload_size;
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    uavcan_register_Access_Request_1_0_deserialize_(&_request_msg, payload, &payload_len);

    return _registers.getRegisterIndexByRegisterName(_request_msg.name.name.elements, _request_msg.name.name.count);
}

void CyphalRegisterAccessRequest::makeResponse(const CanardRxTransfer* transfer, int8_t reg_index)
{
    constexpr uint8_t EMPTY_TAG = 0;
    constexpr uint8_t STRING_TAG = 1;
    constexpr uint8_t NATURAL16_TAG = 10;

    _transfer_metadata.remote_node_id = transfer->metadata.remote_node_id;
    _transfer_metadata.transfer_id = transfer->metadata.transfer_id;

    // The write operation is performed first
    if (_request_msg.value._tag_ == NATURAL16_TAG && _request_msg.value.natural16.value.count > 0) {
        _registers.setPortIdByIndex(reg_index, _request_msg.value.natural16.value.elements[0]);
    }

    // On the next step the register will be read regardless of the outcome of the write operation
    if (reg_index < 0 || reg_index >= CyphalRegisters::NUMBER_OF_REGISTERS) {
        response_msg.value._tag_ = EMPTY_TAG;
        // response_msg.value.natural16.value.elements[0] = CyphalRegisters::CYPHAL_INVALID_REGISTER_VALUE;
    } else if (reg_index < CyphalRegisters::NUMBER_OF_INTEGER_REGISTERS) {
        response_msg.value._tag_ = NATURAL16_TAG;
        response_msg.value.natural16.value.count = 1;
        response_msg.value.natural16.value.elements[0] = _registers.getPortIdByIndex(reg_index);
    } else {
        response_msg.value._tag_ = STRING_TAG;
        auto str = string_registers[reg_index - CyphalRegisters::NUMBER_OF_INTEGER_REGISTERS];
        response_msg.value._string.value.count = strlen(str);
        memcpy(response_msg.value._string.value.elements, str, response_msg.value._string.value.count);
    }

    /// @note: It is not enough memory on stack, so use buffer as static
    static uint8_t buf[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
    int32_t result = uavcan_register_Access_Response_1_0_serialize_(&response_msg, buf, &buf_size);
    if (NUNAVUT_SUCCESS == result) {
        push_response(buf_size, buf);
    }
}


/**
 * @note uavcan.register.List.1.0
 */
void CyphalRegisterListRequest::subscribe()
{
    subscribeOnRequest(uavcan_register_List_Request_1_0_EXTENT_BYTES_);
}

void CyphalRegisterListRequest::handler(const CanardRxTransfer* transfer)
{
    auto index = parseRequest(transfer);
    makeResponse(transfer, index);
}

uint16_t CyphalRegisterListRequest::parseRequest(const CanardRxTransfer* transfer)
{
    auto payload_len = transfer->payload_size;
    const uint8_t* payload = static_cast<const uint8_t*>(transfer->payload);
    uavcan_register_List_Request_1_0 request_msg = {};
    uavcan_register_List_Request_1_0_deserialize_(&request_msg, payload, &payload_len);
    return request_msg.index;
}

void CyphalRegisterListRequest::makeResponse(const CanardRxTransfer* transfer, uint16_t index)
{
    _transfer_metadata.remote_node_id = transfer->metadata.remote_node_id;
    _transfer_metadata.transfer_id = transfer->metadata.transfer_id;

    /// No need to manually clean elements because we explicitly get buffer size
    _response_msg.name.name.count = _registers.getRegisterNameByIndex(index, _response_msg.name.name.elements);

    /// It is not enough memory on stack, so use buffer as static
    /// No need to manually clean it because we explicitly get buffer size
    static uint8_t buf[uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];
    size_t buf_size = uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

    int32_t result = uavcan_register_List_Response_1_0_serialize_(&_response_msg, buf, &buf_size);
    if (NUNAVUT_SUCCESS == result) {
        push_response(buf_size, buf);
    }
}

#endif // HAL_ENABLE_CYPHAL_DRIVERS
