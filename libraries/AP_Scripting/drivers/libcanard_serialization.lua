function cast_float_to_int32(float)
  return string.unpack(">i4", string.pack(">f", float))
end

function cast_int32_to_float(int)
  return string.unpack(">f", string.pack(">i4", int))
end

function cast_native_float_to_float16(origin_native_float)
  local ROUND_MASK = 0xFFFFF000
  local MAGIC_FLOAT = cast_int32_to_float(15 << 23)

  local integer_representation = cast_float_to_int32(origin_native_float)
  integer_representation = integer_representation & ROUND_MASK
  local new_float = cast_int32_to_float(integer_representation) * MAGIC_FLOAT
  local new_float_int32 = cast_float_to_int32(new_float) + 4096
  local int16 = new_float_int32 >> 13

  return int16
end


local function test_cast_native_float_to_float16()
  assert_eq(0, cast_native_float_to_float16(0.0))
  assert_eq(11878, cast_native_float_to_float16(0.1))
  assert_eq(15155, cast_native_float_to_float16(0.9))
  assert_eq(15360, cast_native_float_to_float16(1.0))
end

if not pcall(debug.getlocal, 4, 1) then
  print('Unit tests:')
  require 'libcanard_assets'
  test_cast_native_float_to_float16()
end
