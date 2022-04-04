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

#include "AP_UAVCAN_V1.h"

#if HAL_ENABLE_LIBUAVCAN_V1_DRIVERS

#include <AP_CANManager/AP_CANManager.h>
#include <SRV_Channel/SRV_Channel.h>

#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>


extern const AP_HAL::HAL& hal;


// table of user settable CAN bus parameters
const AP_Param::GroupInfo AP_UAVCAN_V1::var_info[] = {
    // @Param: NODE
    // @DisplayName: UAVCAN node that is used for this network
    // @Description: UAVCAN node should be set implicitly
    // @Range: 1 250
    // @User: Advanced
    AP_GROUPINFO("NODE",    1, AP_UAVCAN_V1, _parameters_table[0], 10),

    // @Param: NOTE
    // @DisplayName: Note response subject id
    // @Description: Note response subject id
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("NOTE",    2, AP_UAVCAN_V1, _parameters_table[1], 0),

    // @Param: SP
    // @DisplayName: Setpoint subject id
    // @Description: Setpoint subject id
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("SP",      3, AP_UAVCAN_V1, _parameters_table[2], 0),

    // @Param: READ
    // @DisplayName: Readiness subject id
    // @Description: reg.udral.service.common.Readiness_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("READ",    4, AP_UAVCAN_V1, _parameters_table[3], 0),


    // @Param: EH1
    // @DisplayName: ESC Heartbeat subject id
    // @Description: reg.udral.service.common.Heartbeat_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("EH1",     5, AP_UAVCAN_V1, _parameters_table[4], 0),

    // @Param: FB1
    // @DisplayName: Feedback subject id
    // @Description: reg.udral.service.actuator.common.Feedback_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("FB1",     6, AP_UAVCAN_V1, _parameters_table[5], 0),

    // @Param: POW1
    // @DisplayName: PowerTs subject id
    // @Description: reg.udral.physics.electricity.PowerTs_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("POW1",    7, AP_UAVCAN_V1, _parameters_table[6], 0),

    // @Param: ST1
    // @DisplayName: Status subject id
    // @Description: reg.udral.service.actuator.common.Status_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("ST1",     8, AP_UAVCAN_V1, _parameters_table[7], 0),

    // @Param: DYN1
    // @DisplayName: Dynamics subject id
    // @Description: reg.udral.physics.dynamics.rotation.PlanarTs_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("DYN1",    9, AP_UAVCAN_V1, _parameters_table[8], 0),


    // @Param: EH2
    // @DisplayName: ESC Heartbeat subject id
    // @Description: reg.udral.service.common.Heartbeat_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("EH2",     10, AP_UAVCAN_V1, _parameters_table[9], 0),

    // @Param: FB2
    // @DisplayName: Feedback subject id
    // @Description: reg.udral.service.actuator.common.Feedback_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("FB2",     11, AP_UAVCAN_V1, _parameters_table[10], 0),

    // @Param: POW2
    // @DisplayName: PowerTs subject id
    // @Description: reg.udral.physics.electricity.PowerTs_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("POW2",    12, AP_UAVCAN_V1, _parameters_table[11], 0),

    // @Param: ST2
    // @DisplayName: Status subject id
    // @Description: reg.udral.service.actuator.common.Status_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("ST2",     13, AP_UAVCAN_V1, _parameters_table[12], 0),

    // @Param: DYN2
    // @DisplayName: Dynamics subject id
    // @Description: reg.udral.physics.dynamics.rotation.PlanarTs_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("DYN2",    14, AP_UAVCAN_V1, _parameters_table[13], 0),


    // @Param: EH3
    // @DisplayName: ESC Heartbeat subject id
    // @Description: reg.udral.service.common.Heartbeat_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("EH3",     15, AP_UAVCAN_V1, _parameters_table[14], 0),

    // @Param: FB3
    // @DisplayName: Feedback subject id
    // @Description: reg.udral.service.actuator.common.Feedback_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("FB3",     16, AP_UAVCAN_V1, _parameters_table[15], 0),

    // @Param: POW3
    // @DisplayName: PowerTs subject id
    // @Description: reg.udral.physics.electricity.PowerTs_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("POW3",    17, AP_UAVCAN_V1, _parameters_table[16], 0),

    // @Param: ST3
    // @DisplayName: Status subject id
    // @Description: reg.udral.service.actuator.common.Status_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("ST3",     18, AP_UAVCAN_V1, _parameters_table[17], 0),

    // @Param: DYN3
    // @DisplayName: Dynamics subject id
    // @Description: reg.udral.physics.dynamics.rotation.PlanarTs_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("DYN3",    19, AP_UAVCAN_V1, _parameters_table[18], 0),


    // @Param: EH4
    // @DisplayName: ESC Heartbeat subject id
    // @Description: reg.udral.service.common.Heartbeat_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("EH4",     20, AP_UAVCAN_V1, _parameters_table[19], 0),

    // @Param: FB4
    // @DisplayName: Feedback subject id
    // @Description: reg.udral.service.actuator.common.Feedback_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("FB4",     21, AP_UAVCAN_V1, _parameters_table[20], 0),

    // @Param: POW4
    // @DisplayName: PowerTs subject id
    // @Description: reg.udral.physics.electricity.PowerTs_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("POW4",    22, AP_UAVCAN_V1, _parameters_table[21], 0),

    // @Param: ST4
    // @DisplayName: Status subject id
    // @Description: reg.udral.service.actuator.common.Status_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("ST4",     23, AP_UAVCAN_V1, _parameters_table[22], 0),

    // @Param: DYN4
    // @DisplayName: Dynamics subject id
    // @Description: reg.udral.physics.dynamics.rotation.PlanarTs_0_1
    // @Range: 0 6143
    // @User: Advanced
    AP_GROUPINFO("DYN4",    24, AP_UAVCAN_V1, _parameters_table[23], 0),

    AP_GROUPEND
};


///< Base must be aligned for O1HEAP_ALIGNMENT byes. Otherwise o1heapInit may fail!
uint8_t base[UAVCAN_HEAP_SIZE] __attribute__ ((aligned (O1HEAP_ALIGNMENT)));

static O1HeapInstance* my_allocator;


static void* memAllocate(CanardInstance* const canard_, const size_t amount);
static void memFree(CanardInstance* const canard_, void* const pointer);


void* memAllocate(CanardInstance* const canard_, const size_t amount)
{
    (void) canard_;
    return o1heapAllocate(my_allocator, amount);
}
void memFree(CanardInstance* const canard_, void* const pointer)
{
    (void) canard_;
    o1heapFree(my_allocator, pointer);
}



AP_UAVCAN_V1 *AP_UAVCAN_V1::get_uavcan(uint8_t driver_index)
{
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_driver_type(driver_index) != AP_CANManager::Driver_Type_UAVCAN_V1) {
        return nullptr;
    }
    return static_cast<AP_UAVCAN_V1*>(AP::can().get_driver(driver_index));
}

void AP_UAVCAN_V1::init(uint8_t driver_index, bool enable_filters)
{
    if (_initialized) {
        return;
    }

    my_allocator = o1heapInit(base, UAVCAN_HEAP_SIZE);
    if (NULL == my_allocator) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "v1: o1heapInit() failed");
        return;
    }

    _canard = canardInit(&memAllocate, &memFree);
    _canard.node_id = 42;
    _tx_queue = canardTxInit(UAVCAN_TX_QUEUE_FRAME_SIZE, CANARD_MTU_CAN_CLASSIC);

    publisher_manager.init(_canard, _tx_queue);
    subscriber_manager.init(_canard, _tx_queue);

    _registers.init(subscriber_manager, _canard, _tx_queue);
    _esc_controller.init(subscriber_manager, publisher_manager, _canard, _tx_queue);

    bool is_thread_created = hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_UAVCAN_V1::loop, void),
                             _thread_name,
                             UAVCAN_STACK_SIZE,
                             AP_HAL::Scheduler::PRIORITY_CAN,
                             0);
    if (!is_thread_created) {
        return;
    }

    _initialized = true;
}

void AP_UAVCAN_V1::loop(void)
{
    while (true) {
        hal.scheduler->delay_microseconds(500); // it should be enough to handle max frame rate

        // do nothing until interface initialization is complete
        if (_can_iface == nullptr) {
            continue;
        }

        spinReceive();
        spinTransmit();
        publisher_manager.process_all();

        static uint32_t next_pub_time_ts = 1000;
        if (AP_HAL::millis() > next_pub_time_ts) {
            next_pub_time_ts += 1000;
            set_ESC_status();
        }
    }
}

bool AP_UAVCAN_V1::add_interface(AP_HAL::CANIface* new_can_iface)
{
    if (_can_iface != nullptr) {
        return false;
    }

    _can_iface = new_can_iface;

    if (_can_iface == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "v1: CAN driver not found");
        return false;
    }

    if (!_can_iface->is_initialized()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "v1: Driver not initialized");
        return false;
    }

    _transport_iface.attach_can_iface(_can_iface);
    return true;
}


void AP_UAVCAN_V1::spinReceive()
{
    // Here we want to spin until fifo is not empty (while True), but typically fifo size is 3
    constexpr uint8_t CAN_RECEIVE_FIFO_SIZE = 3;
    for (uint_fast8_t frame_idx = 0; frame_idx < CAN_RECEIVE_FIFO_SIZE; frame_idx++) {
        CanardFrame canard_frame;
        if (!_transport_iface.receive(&canard_frame)) {
            // There is no avaliable data means fifo is empty
            break;
        }

        CanardRxTransfer transfer;
        auto result = canardRxAccept(&_canard, AP_HAL::micros64(), &canard_frame, 0, &transfer, NULL);
        if (result < 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "v1: recv err: %d", result);
            // An error has occurred: either an argument is invalid or we've ran out of memory.
            // It is possible to statically prove that an out-of-memory will never occur for a given
            // application if the heap is sized correctly; for background, refer to the Robson's Proof
            // and the documentation for O1Heap. Reception of an invalid frame is NOT an error.
            break;
        } else if (result == 1) {
            processReceivedTransfer(0, &transfer);
            _canard.memory_free(&_canard, transfer.payload);
        } else {
            // Nothing to do.
            // The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
            // Reception of an invalid frame is NOT reported as an error because it is not an error.
        }
    }
}

void AP_UAVCAN_V1::spinTransmit()
{
    for (const CanardTxQueueItem* ti = NULL; (ti = canardTxPeek(&_tx_queue)) != NULL;) {
        if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > AP_HAL::micros())) {
            if (!_transport_iface.send(ti)) {
                break;
            }
        }
        _canard.memory_free(&_canard, canardTxPop(&_tx_queue, ti));
    }
}


void AP_UAVCAN_V1::processReceivedTransfer(const uint8_t iface_index, const CanardRxTransfer* transfer)
{
    subscriber_manager.process_all(transfer);
}


void AP_UAVCAN_V1::SRV_push_servos()
{
    _esc_controller.SRV_push_servos();
}

void AP_UAVCAN_V1::set_ESC_status()
{
    for (uint_fast8_t esc_index = 0; esc_index < 4; esc_index++) {
        uint16_t telem_data_mask = _esc_controller.get_avaliable_data_mask(esc_index);

        if (telem_data_mask) {
            TelemetryData telemetry_data = {};

            if (telem_data_mask & AP_ESC_Telem_Backend::TelemetryType::CURRENT) {
                telemetry_data.current = _esc_controller.get_current(esc_index);
            }

            if (telem_data_mask & AP_ESC_Telem_Backend::TelemetryType::VOLTAGE) {
                telemetry_data.voltage = _esc_controller.get_voltage(esc_index);
            }

            if (telem_data_mask & AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE) {
                float temperature_kelvin = _esc_controller.get_temperature(esc_index);
                telemetry_data.temperature_cdeg = int16_t((KELVIN_TO_C(temperature_kelvin) * 100));
            }

            update_telem_data(esc_index, telemetry_data, telem_data_mask);
        }

        uint16_t rpm = _esc_controller.get_rpm(esc_index);
        update_rpm(esc_index, rpm);
    }
}

#endif // HAL_ENABLE_LIBUAVCAN_V1_DRIVERS
