

typedef struct {
    uint64_t index: 4;   // 4-bit index of module number (0b0000 - 0b1111)
    uint64_t faults: 60; // bitmask of each fault field
} bms_errors_t;
static_assert(sizeof(bms_errors_t) == sizeof(uint64_t));


static inline uint32_t as_uint(const float32_t x) {
    return *(uint32_t *)&x;
}

static inline float32_t as_float(const uint32_t x) {
    return *(float32_t *)&x;
}

// https://stackoverflow.com/a/60047308
static float32_t half_to_float(const uint16_t x) { // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
    const uint32_t e = (x&0x7C00)>>10; // exponent
    const uint32_t m = (x&0x03FF)<<13; // mantissa
    const uint32_t v = as_uint((float32_t)m)>>23; // evil log2 bit hack to count leading zeros in denormalized format
    return as_float((x&0x8000)<<16 | (e!=0)*((e+112)<<23|m) | ((e==0)&(m!=0))*((v-37)<<23|((m<<(150-v))&0x007FE000))); // sign : normalized : denormalized
}

static uint16_t float_to_half(const float32_t x) { // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
    const uint32_t b = as_uint(x)+0x00001000; // round-to-nearest-even: add last bit after truncated mantissa
    const uint32_t e = (b&0x7F800000)>>23; // exponent
    const uint32_t m = b&0x007FFFFF; // mantissa; in line below: 0x007FF000 = 0x00800000-0x00001000 = decimal indicator flag - initial rounding
    return (b&0x80000000)>>16 | (e>112)*((((e-112)<<10)&0x7C00)|m>>13) | ((e<113)&(e>101))*((((0x007FF000+m)>>(125-e))+1)>>1) | (e>143)*0x7FFF; // sign : normalized : denormalized : saturate
}

static void bms_send_cells(void)
{
    // 1. Raw voltages. Send 16 bit
    // raw_cell_voltage_a_0
    // raw_cell_voltage_a_1
    // raw_cell_voltage_a_2
    // raw_cell_voltage_a_3
    // ...
    // raw_cell_voltage_f_3

    // 2. min/max
}
