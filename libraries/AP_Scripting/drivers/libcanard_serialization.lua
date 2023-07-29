function castFloatToInt32(float)
  return string.unpack(">i4", string.pack(">f", float))
end

function castInt32ToFloat(int)
  return string.unpack(">f", string.pack(">i4", int))
end

function castNativeFloatToFloat16(origin_float)
  local ROUND_MASK = 0xFFFFF000
  local MAGIC_FLOAT = castInt32ToFloat(15 << 23)

  local integer_representation = castFloatToInt32(origin_float)
  integer_representation = integer_representation & ROUND_MASK
  local new_float = castInt32ToFloat(integer_representation) * MAGIC_FLOAT
  local new_float_int32 = castFloatToInt32(new_float) + 4096
  local int16 = new_float_int32 >> 13

  return int16
end


local function test_castNativeFloatToFloat16()
  assert_eq(0, castNativeFloatToFloat16(0.0))
  assert_eq(11878, castNativeFloatToFloat16(0.1))
  assert_eq(15155, castNativeFloatToFloat16(0.9))
  assert_eq(15360, castNativeFloatToFloat16(1.0))
end

if not pcall(debug.getlocal, 4, 1) then
  print('Unit tests:')
  require 'libcanard_assets'
  test_castNativeFloatToFloat16()
end
