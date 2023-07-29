require 'libcanard_serialization'
require 'libcanard_crc16'

function array_serialize(setpoints, motors_amount, payload)
  payload[1] = motors_amount

  for motor_num = 1, motors_amount do
    setpoint = setpoints[motor_num]
    if (setpoint ~= nil) then
      payload[motor_num << 1] = setpoint % 256
      payload[(motor_num << 1) + 1] = (setpoint >> 8) % 256
    end
  end

  return 1 + 2 * motors_amount
end

function vector_serialize(setpoints, motors_amount, payload)
  for motor_idx = 0, motors_amount - 1 do
    setpoint_f16 = cast_native_float_to_float16(setpoints[motor_idx + 1])
    if (setpoint ~= nil) then
      payload[(motor_idx << 1) + 1] = setpoint_f16 % 256
      payload[(motor_idx << 1) + 2] = (setpoint_f16 >> 8) % 256
    end
  end

  return motors_amount * 2
end
