

typedef struct {
    uint64_t index: 4;   // 4-bit index of module number (0b0000 - 0b1111)
    uint64_t faults: 60; // bitmask of each fault field
} bms_errors_t;
static_assert(sizeof(bms_errors_t) == sizeof(uint64_t));

