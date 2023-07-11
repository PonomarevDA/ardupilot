-- Cyphal
local driver1 = CAN:get_device(20)
assert(driver1 ~= nil, 'No scripting CAN interfaces found')

local node_id = 42

local HEARTBEAT_PORT_ID = 7509
local heartbeat_transfer_id = 0
local next_heartbeat_pub_time_ms = 1000

local readiness_port_id = 2001
local readiness_transfer_id = 0
local next_readiness_pub_time_ms = 1000

local setpoint_port_id = 2000
local setpoint_transfer_id = 0

local feedback_port_id = 2002


local PARAM_TABLE_KEY = 42
assert(param:add_table(PARAM_TABLE_KEY, "CYP_", 1), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ENABLE', 1), 'could not add CYP_ENABLE param')

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
  frame = driver1:read_frame()
  while frame do
    port_id = parse_frame(frame)
    if port_id == HEARTBEAT_PORT_ID then
    elseif port_id == feedback_port_id then
      esc_rpm_callback(frame)
    end
    frame = driver1:read_frame()
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
  msg:data(7, create_tail_byte_for_single_frame_msg(heartbeat_transfer_id))
  msg:dlc(8)
  driver1:write_frame(msg, 1000000)
  heartbeat_transfer_id = increment_transfer_id(heartbeat_transfer_id)
end

function process_readiness()
  -- uint2 value

  if next_readiness_pub_time_ms >= millis() then
    return
  end
  next_readiness_pub_time_ms = millis() + 100

  msg = CANFrame()
  msg:id( get_msg_id(readiness_port_id, node_id) )

  msg:data(0, 3) -- engaged
  msg:data(1, create_tail_byte_for_single_frame_msg(readiness_transfer_id))
  msg:dlc(2)
  driver1:write_frame(msg, 1000000)
  readiness_transfer_id = increment_transfer_id(readiness_transfer_id)
end

function send_setpoint()
  -- y sub 2000:uavcan.primitive.scalar.Integer16
  -- channels for quadcopter: 33-36

  msg = CANFrame()
  msg:id(get_msg_id(setpoint_port_id, node_id))

  pwm = SRV_Channels:get_output_pwm(33)
  if (pwm ~= nil) then
    msg:data(0, pwm % 256)
    msg:data(1, (pwm >> 8) % 256)
  end

  msg:data(2, create_tail_byte_for_single_frame_msg(setpoint_transfer_id))
  msg:dlc(3)
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

function esc_rpm_callback(frame)
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

function create_tail_byte_for_single_frame_msg(transfer_id)
  return 224 + transfer_id
end

function parse_id(id)
  service_not_message = (id >> 25) % 2
  if service_not_message == 0 then
    port_id = (id >> 8) % 8192
  else
    port_id = 8191
  end
  return port_id
end
-- End of the specification related section


-- Start of the unit tests section
function assert_eq(first_int, second_int)
  if first_int ~= second_int then
    gcs:send_text(5, string.format("Assert error %i ~= %i", first_int, second_int))
  else
    gcs:send_text(6, string.format("Assert has been passed"))
  end
end

function test_parse_id()
  assert_eq(8191, parse_id(34067071))   -- srv, skip for a while
  assert_eq(2002, parse_id(512639))     -- msg node_id=127, subject_id=2002 (synthetic)
  assert_eq(2002, parse_id(275239551))  -- msg node_id=127, subject_id=2002 (real example)
end
-- End of the unit tests section


-- Entry point
gcs:send_text(5, "LUA Cyphal enabled!")
gcs:send_text(5, "LUA Cyphal unit tests enabled!")
test_parse_id()
return update()
