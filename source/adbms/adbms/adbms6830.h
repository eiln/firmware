
#ifndef __ADBMS_6830_H__
#define __ADBMS_6830_H__

#include "main.h"
#include <assert.h>

extern uint8_t  txData[TOTAL_AD68][DATA_LEN];
extern uint8_t  rxData[TOTAL_AD68][DATA_LEN];

typedef struct
{
    uint8_t     pwm1    :4;
    uint8_t     pwm2    :4;

    uint8_t     pwm3    :4;
    uint8_t     pwm4    :4;

    uint8_t     pwm5    :4;
    uint8_t     pwm6    :4;

    uint8_t     pwm7    :4;
    uint8_t     pwm8    :4;

    uint8_t     pwm9    :4;
    uint8_t     pwm10   :4;

    uint8_t     pwm11   :4;
    uint8_t     pwm12   :4;
} ad68_pwma_t;

typedef struct
{
    uint8_t     pwm13   :4;
    uint8_t     pwm14   :4;

    uint8_t     pwm15   :4;
    uint8_t     pwm16   :4;

    uint32_t    rsv     :32;
} ad68_pwmb_t;

typedef struct
{
    uint8_t cfa_Tx[DATA_LEN];
    uint8_t cfb_Tx[DATA_LEN];
    ad68_pwma_t pwma;
    ad68_pwmb_t pwmb;
} ic_ad68_t;

uint32_t adbms_checkalive(void);
float getVoltage(int data);

void bms_init(void);
void bms_writeConfigA(void);
void bms_writeConfigB(void);

void bms_writePwm(uint8_t pwm[TOTAL_AD68][TOTAL_CELL]);
void bms_startDischarge(uint8_t pwm[TOTAL_AD68][TOTAL_CELL]);

void adBms6830_Adcv(uint8_t rd, uint8_t cont, uint8_t dcp, uint8_t rstf, uint8_t owcs);
void adBms6830_Adsv(uint8_t cont, uint8_t dcp, uint8_t owcs);
void adBms6830_Adax(uint8_t owaux, uint8_t pup, uint8_t ch);

void bms_readCellVoltages(void);
void bms_readAuxVoltages(void);
void bms_checkCellVoltagesStatC(void);
void bms_readSVoltages(void);

void bms_openWireCheck(void);
void bms_getAuxMeasurement(void);
void bms_printVoltage(float vArr[16]);
void bms_readStatus(void);
extern ic_ad68_t ic_ad68[TOTAL_AD68];

typedef enum __attribute__ ((__packed__))
{
    ADCV_RD_OFF = 0x0,
    ADCV_RD_ON  = 0x1,
} adcv_rd;
static_assert(sizeof(adcv_rd) == sizeof(uint8_t));

typedef enum __attribute__ ((__packed__))
{
    ADCV_CONT_SINGLE = 0x0,
    ADCV_CONT_CONTINUOUS = 0x1,
} adcv_cont;
static_assert(sizeof(adcv_cont) == sizeof(uint8_t));

/*!
*  \enum DCP
* DCP: Discharge permitted.
*/
/* Discharge permitted */
typedef enum __attribute__ ((__packed__))
{
    DCP_OFF = 0X0,
    DCP_ON = 0X1,
} adcv_dcp;
static_assert(sizeof(adcv_dcp) == sizeof(uint8_t));

// Open wire c/s.
// current source
typedef enum __attribute__ ((__packed__))
{
    OW_OFF_ALL_CH = 0x0,
    OW_ON_EVEN_CH = 0x1,
    OW_ON_ODD_CH  = 0x2,
    OW_ON_ALL_CH  = 0x3,
} adcv_owcs;
static_assert(sizeof(adcv_owcs) == sizeof(uint8_t));

// OW_AUX: Open wire Aux.
typedef enum __attribute__ ((__packed__))
{
    AUX_OW_OFF = 0X0,
    AUX_OW_ON  = 0X1,
} adcv_owaux;
static_assert(sizeof(adcv_owaux) == sizeof(uint8_t));

// PUP: Pull Down current during aux conversion.
/* Pull Down current during aux conversion (if OW = 1) */
typedef enum __attribute__ ((__packed__))
{
    PUP_DOWN = 0X0,
    PUP_UP = 0X1,
} adcv_pup;
static_assert(sizeof(adcv_pup) == sizeof(uint8_t));

// RSTF: Reset Filter.
/* Pull Down current during aux conversion (if OW = 1) */
typedef enum __attribute__ ((__packed__))
{
    RSTF_OFF = 0x0,
    RSTF_ON = 0x1,
} adcv_rstf;
static_assert(sizeof(adcv_rstf) == sizeof(uint8_t));

// ERR: Inject error is spi read out.
/* Pull Down current during aux conversion (if OW = 1) */
typedef enum __attribute__ ((__packed__))
{
    WITHOUT_ERR = 0x0,
    WITH_ERR = 0x1,
} adcv_err;
static_assert(sizeof(adcv_err) == sizeof(uint8_t));

#define AUX_ALL 0  // select all 10 GPIOs

#endif // __ADBMS_6830_H__
