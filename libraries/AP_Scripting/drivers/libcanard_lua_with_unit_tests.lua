local CRC16_LOOKUP = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C,
  0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318,
  0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4,
  0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630,
  0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4,
  0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969,
  0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12, 0xDBFD, 0xCBDC, 0xFBBF,
  0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A, 0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13,
  0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9,
  0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046,
  0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2,
  0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2,
  0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E,
  0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E,
  0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1,
  0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07,
  0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9,
  0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
}

local UNUSED_PORT_ID = 65535

function crc16_add_byte(prev_crc, byte_value)
  return ((prev_crc << 8) & 0xFFFF) ~ CRC16_LOOKUP[((prev_crc >> 8) ~ byte_value) + 1]
end

function calc_crc16(byte_array, num_bytes)
  start_byte = 1
  local crc = 0xFFFF
  for i = 1, num_bytes do
    local byte_value = byte_array[i] & 0xFF
    crc = crc16_add_byte(crc, byte_value)
  end
  return crc
end

function get_number_of_frames_by_payload_size(number_of_bytes)
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

function parse_id(id)
  service_not_message = (id >> 25) % 2
  if service_not_message == 0 then
    port_id = (id >> 8) % 8192
  else
    port_id = UNUSED_PORT_ID
  end
  return port_id
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


function create_setpoint_payload(payload, motor_num)
  payload_size = 1
  payload[payload_size] = 8

  for motor_idx = 0, motor_num - 1 do
    pwm_duration_us = 1000 + motor_idx * 100
    if (pwm_duration_us ~= nil) then
      payload[2 + motor_idx * 2] = pwm_duration_us % 256
      payload[3 + motor_idx * 2] = (pwm_duration_us >> 8) % 256
    end
    payload_size = payload_size + 2
  end

  return payload_size
end

function serialize_payload_to_can_buffer(buffer, payload, payload_size, transfer_id)
  number_of_frames = get_number_of_frames_by_payload_size(payload_size)
  buffer_size = 0
  tail_byte_counter = 0
  for payload_idx = 1, payload_size do
    if payload_idx % 7 == 1 and buffer_size ~= 0 then
      buffer_size = buffer_size + 1
      tail_byte_counter = tail_byte_counter + 1
      buffer[buffer_size] = create_tail_byte(tail_byte_counter, number_of_frames, transfer_id)
    end
    buffer_size = buffer_size + 1
    buffer[buffer_size] = payload[payload_idx]
  end

  if number_of_frames > 1 then
    crc = calc_crc16(payload, payload_size)
    buffer_size = buffer_size + 1
    buffer[buffer_size] = crc >> 8
    buffer_size = buffer_size + 1
    buffer[buffer_size] = crc % 256
    buffer_size = buffer_size + 1
    tail_byte_counter = tail_byte_counter + 1
    buffer[buffer_size] = create_tail_byte(tail_byte_counter, number_of_frames, transfer_id)
  end

  return buffer_size
end

-- Start of the unit tests section
function assert_eq(first_int, second_int, msg)
  if first_int ~= second_int then
    print("Fail", first_int, second_int)
  elseif msg ~= nil then
    print("Good", first_int)
  end
end

function test_crc16()
  assert_eq(62800, crc16_add_byte(0xFFFF, 0xAA))
  assert_eq(34620, crc16_add_byte(62800, 0x42))

  local byte_array = {0xAA, 0x42}
  assert_eq(34620, calc_crc16(byte_array, 2))
end

function test_parse_id()
  assert_eq(UNUSED_PORT_ID, parse_id(34067071)) -- srv, skip for a while
  assert_eq(2002, parse_id(512639))             -- msg node_id=127, subject_id=2002 (synthetic)
  assert_eq(2002, parse_id(275239551))          -- msg node_id=127, subject_id=2002 (real example)
end

function test_get_number_of_frames_by_payload_size()
  assert_eq(1, get_number_of_frames_by_payload_size(7))
  assert_eq(2, get_number_of_frames_by_payload_size(8))
  assert_eq(2, get_number_of_frames_by_payload_size(12))
  assert_eq(3, get_number_of_frames_by_payload_size(13))
  assert_eq(3, get_number_of_frames_by_payload_size(19))
  assert_eq(4, get_number_of_frames_by_payload_size(20))
end

function test_create_tail_byte()
  assert_eq(244, create_tail_byte(1, 1, 20))
  assert_eq(174, create_tail_byte(1, 2, 14))
  assert_eq(88, create_tail_byte(2, 2, 24))
  assert_eq(13, create_tail_byte(2, 3, 13))
end

function test_array_serialization()
  payload = {}
  payload_size = create_setpoint_payload(payload, 8)
  assert_eq(17, payload_size)

  payload = {
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00
  }
  expected_buffer = {
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xA4,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04,
    0x00, 0x00, 0x00, 0x40, 0xFC, 0x64
  }
  transfer_id = 4
  buffer = {}
  buffer_size = serialize_payload_to_can_buffer(buffer, payload, payload_size, transfer_id)
  assert_eq(22, buffer_size)
  for idx = 1, buffer_size do
    assert_eq(expected_buffer[idx], buffer[idx])
  end

  payload = {
    0x08, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00,
    0x35, 0x00, 0x36, 0x00, 0x37, 0x00, 0x37,
    0x00, 0x37, 0x00
  }
  expected_buffer = {
    0x08, 0x32, 0x00, 0x33, 0x00, 0x34, 0x00, 0xAC,
    0x35, 0x00, 0x36, 0x00, 0x37, 0x00, 0x37, 0x0C,
    0x00, 0x37, 0x00, 0x48, 0xC3, 0x6C
  }
  transfer_id = 12
  buffer = {}
  buffer_size = serialize_payload_to_can_buffer(buffer, payload, payload_size, transfer_id)
  assert_eq(22, buffer_size)
  for idx = 1, buffer_size do
    assert_eq(expected_buffer[idx], buffer[idx])
  end
end
-- End of the unit tests section


test_crc16()
test_parse_id()
test_get_number_of_frames_by_payload_size()
test_create_tail_byte()
test_array_serialization()
print("Done")
