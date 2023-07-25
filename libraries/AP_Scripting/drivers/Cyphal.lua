-- Cyphal
local driver1 = CAN:get_device(20)
assert(driver1 ~= nil, 'No scripting CAN interfaces found')

local PARAM_TABLE_KEY = 42
assert(param:add_table(PARAM_TABLE_KEY, "CYP_", 6), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add CYP_ENABLE param')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'NODE_ID', 127), 'could not add CYP_NODE_ID param')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'TESTS', 0), 'could not add CYP_TESTS param')
assert(param:add_param(PARAM_TABLE_KEY, 4, 'SP', 65535), 'could not add CYP_SP param')
assert(param:add_param(PARAM_TABLE_KEY, 5, 'RD', 65535), 'could not add CYP_RD param')
assert(param:add_param(PARAM_TABLE_KEY, 6, 'FB', 65535), 'could not add CYP_FB param')

local param_cyp_enable = Parameter("CYP_ENABLE")  -- 1 = enabled, 0 = disabled
local param_cyp_tests = Parameter("CYP_TESTS")    -- 1 = enabled, 0 = disabled
local node_id = Parameter("CYP_NODE_ID"):get()

local HEARTBEAT_PORT_ID = 7509
local heartbeat_transfer_id = 0
local next_heartbeat_pub_time_ms = 1000

local readiness_port_id = Parameter("CYP_RD"):get()
local readiness_transfer_id = 0
local next_readiness_pub_time_ms = 1000

local setpoint_port_id = Parameter("CYP_SP"):get()
local setpoint_transfer_id = 0

local feedback_port_id = Parameter("CYP_FB"):get()

-- Constants
local UNUSED_PORT_ID = 65535
local MAX_PORT_ID = 8191
local MOTOR_1_FUNC_IDX = 33
local NUMBER_OF_MOTORS = 3

local READINESS_STANDBY = 2
local READINESS_ENGAGED = 3


local next_log_time = 1000
local loop_counter = 0

function update()
  spin_recv()
  process_heartbeat()
  process_readiness()
  send_setpoint()

  check_perfomance()

  return update, 4 -- ms
end

function spin_recv()
  for _ = 0, 4 do
    frame = driver1:read_frame()
    if not frame then
      return
    end

    port_id = parse_frame(frame)
    if port_id == feedback_port_id then
      esc_rpm_callback()
    end
  end
end

function process_heartbeat()
  -- uint32 uptime # [second]
  -- uint8 health
  -- uint8 mode
  -- uint8 vsscs

  if next_heartbeat_pub_time_ms >= millis() then
    return
  end
  next_heartbeat_pub_time_ms = millis() + 500

  msg = CANFrame()
  msg:id(get_msg_id(HEARTBEAT_PORT_ID, node_id))

  local now_sec = (millis() / 1000)
  msg:data(0, (now_sec & 255):toint())
  msg:data(1, ((now_sec >> 8) & 255):toint())
  msg:data(2, ((now_sec >> 16) & 255):toint())
  msg:data(3, ((now_sec >> 24) & 255):toint())
  msg:data(7, create_tail_byte(1, 1, heartbeat_transfer_id))
  msg:dlc(8)
  driver1:write_frame(msg, 1000000)
  heartbeat_transfer_id = increment_transfer_id(heartbeat_transfer_id)
end

function process_readiness()
  -- y sub 2343:reg.udral.service.common.Readiness
  if readiness_port_id > MAX_PORT_ID then
    return
  end
  if next_readiness_pub_time_ms >= millis() then
    return
  end
  next_readiness_pub_time_ms = millis() + 100

  msg = CANFrame()
  msg:id( get_msg_id(readiness_port_id, node_id) )

  if arming:is_armed() then
    msg:data(0, READINESS_ENGAGED)
  else
    msg:data(0, READINESS_STANDBY)
  end

  msg:data(1, create_tail_byte(1, 1, readiness_transfer_id))
  msg:dlc(2)
  driver1:write_frame(msg, 1000000)
  readiness_transfer_id = increment_transfer_id(readiness_transfer_id)
end

function send_setpoint()
  -- y sub 2342:uavcan.primitive.array.Integer16
  -- y sub 2342:reg.udral.service.actuator.common.sp.Vector8
  -- channels for quadcopter: 33-36 - pwm duration from 1000 us to 2000 us

  msg = CANFrame()
  msg:id(get_msg_id(setpoint_port_id, node_id))

  msg:data(0, NUMBER_OF_MOTORS)
  for motor_idx = 0, NUMBER_OF_MOTORS - 1 do
    pwm_duration_us = SRV_Channels:get_output_pwm(MOTOR_1_FUNC_IDX + motor_idx)
    if (pwm_duration_us ~= nil) then
      msg:data(motor_idx * 2 + 1, pwm_duration_us % 256)
      msg:data(motor_idx * 2 + 2, (pwm_duration_us >> 8) % 256)
    end
  end

  msg:data(7, create_tail_byte(1, 1, setpoint_transfer_id))
  msg:dlc(8)
  driver1:write_frame(msg, 1000000)

  setpoint_transfer_id = increment_transfer_id(setpoint_transfer_id)
end

function check_perfomance()
  loop_counter = loop_counter + 1
  if next_log_time <= millis() then
    next_log_time = millis() + 5000
    gcs:send_text(6, string.format("LUA loop times: %i", loop_counter))
    loop_counter = 0
  end
end

function esc_rpm_callback()
  esc_telem:update_rpm(0, 100, 1)
  esc_telem:update_rpm(1, 110, 2)
  esc_telem:update_rpm(2, 120, 3)
  esc_telem:update_rpm(3, 130, 5)
end

function parse_frame(frame)
  return parse_id(frame:id_signed())
end

-- Start of the specification related section
function get_msg_id(port, node)
  return uint32_t(2422210560) + port * 256 + node
end

function increment_transfer_id(transfer_id)
  if transfer_id >= 31 then
    return 0
  else
    return transfer_id + 1
  end
end

function get_number_of_frames_by_bytes(number_of_bytes)
  -- =IF(BYTES>0;IF(BYTES>7;CEILING((BYTES+2)/7);1);)
  number_of_frames = 0

  if number_of_bytes <= 7 then
    number_of_frames = 1
  elseif number_of_bytes <= 12 then
    number_of_frames = 2
  elseif number_of_bytes <= 19 then
    number_of_frames = 3
  elseif number_of_bytes <= 26 then
    number_of_frames = 4
  end

  return number_of_frames
end

function create_tail_byte(frame_num, number_of_frames, transfer_id)
  -- transfer_id + toggle_bit (32) + end_of_transfer (64) + start_of_transfer (128)
  tail_byte = transfer_id

  if frame_num == 1 then
    tail_byte = tail_byte + 128
  end
  if frame_num == number_of_frames then
    tail_byte = tail_byte + 64
  end
  if (frame_num % 2) == 1 then
    tail_byte = tail_byte + 32
  end
 
  return tail_byte
end

function parse_id(id)
  service_not_message = (id >> 25) % 2
  if service_not_message == 0 then
    port_id = (id >> 8) % 8192
  else
    port_id = UNUSED_PORT_ID
  end
  return port_id
end
-- End of the specification related section


-- Start of the unit tests section
function assert_eq(first_int, second_int)
  if first_int ~= second_int then
    gcs:send_text(5, string.format("Assert error %i ~= %i", first_int, second_int))
  else
    gcs:send_text(6, string.format("Assert has been passed %i", first_int))
  end
end

function test_parse_id()
  assert_eq(UNUSED_PORT_ID, parse_id(34067071)) -- srv, skip for a while
  assert_eq(2002, parse_id(512639))             -- msg node_id=127, subject_id=2002 (synthetic)
  assert_eq(2002, parse_id(275239551))          -- msg node_id=127, subject_id=2002 (real example)

  assert_eq(1, get_number_of_frames_by_bytes(7))
  assert_eq(2, get_number_of_frames_by_bytes(8))
  assert_eq(2, get_number_of_frames_by_bytes(12))
  assert_eq(3, get_number_of_frames_by_bytes(13))
  assert_eq(3, get_number_of_frames_by_bytes(19))
  assert_eq(4, get_number_of_frames_by_bytes(20))

  assert_eq(244, create_tail_byte(1, 1, 20))
  assert_eq(174, create_tail_byte(1, 2, 14))
  assert_eq(88, create_tail_byte(2, 2, 24))
  assert_eq(13, create_tail_byte(2, 3, 13))
end
-- End of the unit tests section


-- Entry point
if (param_cyp_tests:get() >= 1) then
  gcs:send_text(5, "LUA Cyphal unit tests enabled!")
  test_parse_id()
end

if (param_cyp_enable:get() >= 1) then
  gcs:send_text(5, "LUA Cyphal enabled!")
  return update()
end
