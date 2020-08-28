# Timestamp, it is recommended that every message must have
uint64 timestamp # time since system start (microseconds)

# General data type
int8 s8_value
int16 s16_value
int32 s32_value
int64 s64_value

uint8 u8_value
uint16 u16_value
uint32 u32_value
uint64 u64_value

float32 f32_value
float64 f64_value

# Array
uint32[10] uint32_array # Can be of any type
int8[50] string

# Constant
# Will generate a macro (For C) and a constant in the structure (For C++)
uint32 u32_constant = 10 # Can be of any type
