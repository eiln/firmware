
#include "main.h"
#include "adbms/adbms.h"
#include "math.h"

static void bms_read_cells(void);
static void bms_check_cells(void);
static void bms_send_cells(void);

void bms_monitor_cells(void)
{
    bms_read_cells();
    bms_send_cells(); // Still send
}

static void bms_print_cell_voltages(void)
{
    printf("C-ADC Voltages:\n");
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            printf("Cell %02d: %.4f ", i, data.cell_v_c[ic][i]);
            if (i % 4 == 3)
            printf("\n");
        }
    }
    printf("\n");
}

void bms_monitor_cells_start(void)
{
    adBms6830_Adcv(ADCV_RD_OFF, ADCV_CONT_CONTINUOUS, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
    bms_mDelay(1);
}

static void bms_read_cells(void)
{
    #if 0
    The direct method involves setting the redundancy bit (RD) in an
    ADCV command. In this case, the C-ADCs and S-ADCs are both
    triggered to provide redundancy. After 8 ms, the average results
    of the C-ADCs are compared to the results of the S-ADCs

    If the results do not match within the threshold set by the CTH[2:0] in
    Configuration Register A, the CSxFLT flag is set in the Status
    Register Group C.

    ADSV with DCP = 0, CONT = 1, OW = 0 (redundant check)
    ► ADSV with DCP = 0, CONT = 0, OW = 1 (even open wire
    check)
    ► ADSV with DCP = 0, CONT = 0, OW = 2 (odd open wire
    check)

    C-ADC conversions are usually started once during initialization:
    C-ADCs run in continuous mode, deliver measurement re-
    sults, and feed the IIR filter. No comparison between C-ADC
    and S-ADC results is performed. PWM discharge is ongoing
    and is not affected

    Thus, the whole redundant and open wire diagnostic takes 24
    ms to 32 ms and limits the maximum discharge duty cycle. In
    average, the discharge is inhibited for 0.5 × (32 ms + 24 ms) =
    28 ms. Assuming an FTTI of 100 ms, the maximum discharge duty
    cycle of the ADBMS6830B is limited to 72% (even if the PWM
    was configured to 100%, it is limited to 72% by the diagnostic
    measurements).
    #endif

    // 16 additional ADCs are dedicated to measure the 16 differential
    // inputs (SxP and SxN) synchronously with an input range of 0 V
    // to 5. 5 V and a sampling frequency of ~4 MHz, giving out results
    // every 8 ms.
#if 0
    adBms6830_Adcv(ADCV_RD_OFF, ADCV_CONT_SINGLE, DCP_OFF, RSTF_OFF, OW_ON_EVEN_CH);
    bms_mDelay(1);
    bms_readCellVoltages(); // TODO store in ow slot and compare

    adBms6830_Adcv(ADCV_RD_OFF, ADCV_CONT_SINGLE, DCP_OFF, RSTF_OFF, OW_ON_ODD_CH);
    bms_mDelay(1);
    bms_readCellVoltages();
#endif

    bms_readCellVoltages();

    adBms6830_Adsv(ADCV_CONT_SINGLE, DCP_OFF, OW_OFF_ALL_CH);
    bms_mDelay(8);
    bms_readSVoltages();

#if 0
    adBms6830_Adcv(ADCV_RD_OFF, ADCV_CONT_SINGLE, DCP_OFF, RSTF_OFF, OW_OFF_ALL_CH);
    bms_mDelay(1);
    bms_checkCellVoltagesStatC(); // TODO check STAT
    bms_readCellVoltages();
#endif

    if (bms_any_fault(BMS_ERROR_RXPEC))
    {
        // Bad readings (last checked before cell reading)
        // Discard away bad readings
        bms_error("Bad cell readings! Discarding\n");
        return;
    }

    bms_print_cell_voltages();
    bms_check_cells(); // Only check faults from readings if readings are good

    // TODO Add LPF/EMA if needed
    // TODO transfer cell_v_c to cell_v
}

static void bms_check_cells(void)
{
    // TODO SIMD lol
    bool set;

    #define CELL_REDUN_DELTA_MAX (0.05) // V

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int i = 0; i < TOTAL_CELL; i++)
        {
            float delta = fabsf(data.cell_v_c[ic][i] - data.cell_v_s[ic][i]);
            set = delta > CELL_REDUN_DELTA_MAX;
            BMS_SET_FAULT_CELL_DEBUG(i, BMS_ERROR_CELL_REDUN, delta);
        }
    }
}

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

static void bms_process_cell_readings(void)
{
    // Calculate min/max/average

    float mod_volts[TOTAL_AD68][4]; // min, max, avg, delta
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float min = data.cell_v_c[ic][0];
        float max = data.cell_v_c[ic][0];
        float avg = 0.0f;
        for (int cell = 0; cell < TOTAL_CELL; cell++)
        {
            float volts = data.cell_v_c[ic][cell];
            min = MIN(volts, min);
            max = MAX(volts, max);
            avg += volts;
        }
        avg /= (float)TOTAL_CELL;

        mod_volts[ic][0] = min;
        mod_volts[ic][1] = max;
        mod_volts[ic][2] = avg;
        mod_volts[ic][3] = max - min;
    }
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
