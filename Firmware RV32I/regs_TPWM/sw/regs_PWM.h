// Created with Corsair v1.0.4
#ifndef __REGS_PWM_H
#define __REGS_PWM_H

#define __I  volatile const // 'read only' permissions
#define __O  volatile       // 'write only' permissions
#define __IO volatile       // 'read / write' permissions


#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define PCSR_BASE_ADDR 0x0

// TPWM_config - PWM configuration register
#define PCSR_TPWM_CONFIG_ADDR 0x0
#define PCSR_TPWM_CONFIG_RESET 0x0
typedef struct {
    uint32_t EN : 1; // Enable PWM operation
    uint32_t SEL1 : 2; // Output selection for PWM1
    uint32_t SEL2 : 2; // Output selection for PWM2
    uint32_t RESERVED1 : 2; // Reserved for future use
    uint32_t SEL3 : 2; // Output selection for PWM3
    uint32_t SEL4 : 2; // Output selection for PWM4
    uint32_t NUM : 3; // Number of PWM signals selected
    uint32_t POL : 1; // Polarity of the PWM signal
    uint32_t : 17; // reserved
} pcsr_tpwm_config_t;

// TPWM_config.EN - Enable PWM operation
#define PCSR_TPWM_CONFIG_EN_WIDTH 1
#define PCSR_TPWM_CONFIG_EN_LSB 0
#define PCSR_TPWM_CONFIG_EN_MASK 0x1
#define PCSR_TPWM_CONFIG_EN_RESET 0x0

// TPWM_config.SEL1 - Output selection for PWM1
#define PCSR_TPWM_CONFIG_SEL1_WIDTH 2
#define PCSR_TPWM_CONFIG_SEL1_LSB 1
#define PCSR_TPWM_CONFIG_SEL1_MASK 0x6
#define PCSR_TPWM_CONFIG_SEL1_RESET 0x0

// TPWM_config.SEL2 - Output selection for PWM2
#define PCSR_TPWM_CONFIG_SEL2_WIDTH 2
#define PCSR_TPWM_CONFIG_SEL2_LSB 3
#define PCSR_TPWM_CONFIG_SEL2_MASK 0x18
#define PCSR_TPWM_CONFIG_SEL2_RESET 0x0

// TPWM_config.RESERVED1 - Reserved for future use
#define PCSR_TPWM_CONFIG_RESERVED1_WIDTH 2
#define PCSR_TPWM_CONFIG_RESERVED1_LSB 5
#define PCSR_TPWM_CONFIG_RESERVED1_MASK 0x60
#define PCSR_TPWM_CONFIG_RESERVED1_RESET 0x0

// TPWM_config.SEL3 - Output selection for PWM3
#define PCSR_TPWM_CONFIG_SEL3_WIDTH 2
#define PCSR_TPWM_CONFIG_SEL3_LSB 7
#define PCSR_TPWM_CONFIG_SEL3_MASK 0x180
#define PCSR_TPWM_CONFIG_SEL3_RESET 0x0

// TPWM_config.SEL4 - Output selection for PWM4
#define PCSR_TPWM_CONFIG_SEL4_WIDTH 2
#define PCSR_TPWM_CONFIG_SEL4_LSB 9
#define PCSR_TPWM_CONFIG_SEL4_MASK 0x600
#define PCSR_TPWM_CONFIG_SEL4_RESET 0x0

// TPWM_config.NUM - Number of PWM signals selected
#define PCSR_TPWM_CONFIG_NUM_WIDTH 3
#define PCSR_TPWM_CONFIG_NUM_LSB 11
#define PCSR_TPWM_CONFIG_NUM_MASK 0x3800
#define PCSR_TPWM_CONFIG_NUM_RESET 0x0

// TPWM_config.POL - Polarity of the PWM signal
#define PCSR_TPWM_CONFIG_POL_WIDTH 1
#define PCSR_TPWM_CONFIG_POL_LSB 14
#define PCSR_TPWM_CONFIG_POL_MASK 0x4000
#define PCSR_TPWM_CONFIG_POL_RESET 0x0

// TPWM_prescaler - PWM clock divider
#define PCSR_TPWM_PRESCALER_ADDR 0x4
#define PCSR_TPWM_PRESCALER_RESET 0x1b
typedef struct {
    uint32_t DIV : 32; // Clock divider value for PWM
} pcsr_tpwm_prescaler_t;

// TPWM_prescaler.DIV - Clock divider value for PWM
#define PCSR_TPWM_PRESCALER_DIV_WIDTH 32
#define PCSR_TPWM_PRESCALER_DIV_LSB 0
#define PCSR_TPWM_PRESCALER_DIV_MASK 0xffffffff
#define PCSR_TPWM_PRESCALER_DIV_RESET 0x1b

// TPWM_period1 - PWM1 and PWM2 period register
#define PCSR_TPWM_PERIOD1_ADDR 0x8
#define PCSR_TPWM_PERIOD1_RESET 0xffffffff
typedef struct {
    uint32_t PER1 : 16; // Period value for PWM1
    uint32_t PER2 : 16; // Period value for PWM2
} pcsr_tpwm_period1_t;

// TPWM_period1.PER1 - Period value for PWM1
#define PCSR_TPWM_PERIOD1_PER1_WIDTH 16
#define PCSR_TPWM_PERIOD1_PER1_LSB 0
#define PCSR_TPWM_PERIOD1_PER1_MASK 0xffff
#define PCSR_TPWM_PERIOD1_PER1_RESET 0xffff

// TPWM_period1.PER2 - Period value for PWM2
#define PCSR_TPWM_PERIOD1_PER2_WIDTH 16
#define PCSR_TPWM_PERIOD1_PER2_LSB 16
#define PCSR_TPWM_PERIOD1_PER2_MASK 0xffff0000
#define PCSR_TPWM_PERIOD1_PER2_RESET 0xffff

// TPWM_period2 - PWM3 and PWM4 period register
#define PCSR_TPWM_PERIOD2_ADDR 0xc
#define PCSR_TPWM_PERIOD2_RESET 0xffffffff
typedef struct {
    uint32_t PER3 : 16; // Period value for PWM3
    uint32_t PER4 : 16; // Period value for PWM4
} pcsr_tpwm_period2_t;

// TPWM_period2.PER3 - Period value for PWM3
#define PCSR_TPWM_PERIOD2_PER3_WIDTH 16
#define PCSR_TPWM_PERIOD2_PER3_LSB 0
#define PCSR_TPWM_PERIOD2_PER3_MASK 0xffff
#define PCSR_TPWM_PERIOD2_PER3_RESET 0xffff

// TPWM_period2.PER4 - Period value for PWM4
#define PCSR_TPWM_PERIOD2_PER4_WIDTH 16
#define PCSR_TPWM_PERIOD2_PER4_LSB 16
#define PCSR_TPWM_PERIOD2_PER4_MASK 0xffff0000
#define PCSR_TPWM_PERIOD2_PER4_RESET 0xffff

// TPWM_compare1 - Compare values for PWM1 and PWM2
#define PCSR_TPWM_COMPARE1_ADDR 0x10
#define PCSR_TPWM_COMPARE1_RESET 0xffffffff
typedef struct {
    uint32_t CP1 : 16; // Compare value for PWM1
    uint32_t CP2 : 16; // Compare value for PWM2
} pcsr_tpwm_compare1_t;

// TPWM_compare1.CP1 - Compare value for PWM1
#define PCSR_TPWM_COMPARE1_CP1_WIDTH 16
#define PCSR_TPWM_COMPARE1_CP1_LSB 0
#define PCSR_TPWM_COMPARE1_CP1_MASK 0xffff
#define PCSR_TPWM_COMPARE1_CP1_RESET 0xffff

// TPWM_compare1.CP2 - Compare value for PWM2
#define PCSR_TPWM_COMPARE1_CP2_WIDTH 16
#define PCSR_TPWM_COMPARE1_CP2_LSB 16
#define PCSR_TPWM_COMPARE1_CP2_MASK 0xffff0000
#define PCSR_TPWM_COMPARE1_CP2_RESET 0xffff

// TPWM_compare2 - Compare values for PWM3 and PWM4
#define PCSR_TPWM_COMPARE2_ADDR 0x14
#define PCSR_TPWM_COMPARE2_RESET 0xffffffff
typedef struct {
    uint32_t CP3 : 16; // Compare value for PWM3
    uint32_t CP4 : 16; // Compare value for PWM4
} pcsr_tpwm_compare2_t;

// TPWM_compare2.CP3 - Compare value for PWM3
#define PCSR_TPWM_COMPARE2_CP3_WIDTH 16
#define PCSR_TPWM_COMPARE2_CP3_LSB 0
#define PCSR_TPWM_COMPARE2_CP3_MASK 0xffff
#define PCSR_TPWM_COMPARE2_CP3_RESET 0xffff

// TPWM_compare2.CP4 - Compare value for PWM4
#define PCSR_TPWM_COMPARE2_CP4_WIDTH 16
#define PCSR_TPWM_COMPARE2_CP4_LSB 16
#define PCSR_TPWM_COMPARE2_CP4_MASK 0xffff0000
#define PCSR_TPWM_COMPARE2_CP4_RESET 0xffff

// TPWM_counter1 - PWM counters for PWM1 and PWM2
#define PCSR_TPWM_COUNTER1_ADDR 0x18
#define PCSR_TPWM_COUNTER1_RESET 0x0
typedef struct {
    uint32_t CNT1 : 16; // Current counter value for PWM1
    uint32_t CNT2 : 16; // Current counter value for PWM2
} pcsr_tpwm_counter1_t;

// TPWM_counter1.CNT1 - Current counter value for PWM1
#define PCSR_TPWM_COUNTER1_CNT1_WIDTH 16
#define PCSR_TPWM_COUNTER1_CNT1_LSB 0
#define PCSR_TPWM_COUNTER1_CNT1_MASK 0xffff
#define PCSR_TPWM_COUNTER1_CNT1_RESET 0x0

// TPWM_counter1.CNT2 - Current counter value for PWM2
#define PCSR_TPWM_COUNTER1_CNT2_WIDTH 16
#define PCSR_TPWM_COUNTER1_CNT2_LSB 16
#define PCSR_TPWM_COUNTER1_CNT2_MASK 0xffff0000
#define PCSR_TPWM_COUNTER1_CNT2_RESET 0x0

// TPWM_counter2 - PWM counters for PWM3 and PWM4
#define PCSR_TPWM_COUNTER2_ADDR 0x1c
#define PCSR_TPWM_COUNTER2_RESET 0x0
typedef struct {
    uint32_t CNT3 : 16; // Current counter value for PWM3
    uint32_t CNT4 : 16; // Current counter value for PWM4
} pcsr_tpwm_counter2_t;

// TPWM_counter2.CNT3 - Current counter value for PWM3
#define PCSR_TPWM_COUNTER2_CNT3_WIDTH 16
#define PCSR_TPWM_COUNTER2_CNT3_LSB 0
#define PCSR_TPWM_COUNTER2_CNT3_MASK 0xffff
#define PCSR_TPWM_COUNTER2_CNT3_RESET 0x0

// TPWM_counter2.CNT4 - Current counter value for PWM4
#define PCSR_TPWM_COUNTER2_CNT4_WIDTH 16
#define PCSR_TPWM_COUNTER2_CNT4_LSB 16
#define PCSR_TPWM_COUNTER2_CNT4_MASK 0xffff0000
#define PCSR_TPWM_COUNTER2_CNT4_RESET 0x0


// Register map structure
typedef struct {
    union {
        __IO uint32_t TPWM_CONFIG; // PWM configuration register
        __IO pcsr_tpwm_config_t TPWM_CONFIG_bf; // Bit access for TPWM_CONFIG register
    };
    union {
        __IO uint32_t TPWM_PRESCALER; // PWM clock divider
        __IO pcsr_tpwm_prescaler_t TPWM_PRESCALER_bf; // Bit access for TPWM_PRESCALER register
    };
    union {
        __IO uint32_t TPWM_PERIOD1; // PWM1 and PWM2 period register
        __IO pcsr_tpwm_period1_t TPWM_PERIOD1_bf; // Bit access for TPWM_PERIOD1 register
    };
    union {
        __IO uint32_t TPWM_PERIOD2; // PWM3 and PWM4 period register
        __IO pcsr_tpwm_period2_t TPWM_PERIOD2_bf; // Bit access for TPWM_PERIOD2 register
    };
    union {
        __IO uint32_t TPWM_COMPARE1; // Compare values for PWM1 and PWM2
        __IO pcsr_tpwm_compare1_t TPWM_COMPARE1_bf; // Bit access for TPWM_COMPARE1 register
    };
    union {
        __IO uint32_t TPWM_COMPARE2; // Compare values for PWM3 and PWM4
        __IO pcsr_tpwm_compare2_t TPWM_COMPARE2_bf; // Bit access for TPWM_COMPARE2 register
    };
    union {
        __I uint32_t TPWM_COUNTER1; // PWM counters for PWM1 and PWM2
        __I pcsr_tpwm_counter1_t TPWM_COUNTER1_bf; // Bit access for TPWM_COUNTER1 register
    };
    union {
        __I uint32_t TPWM_COUNTER2; // PWM counters for PWM3 and PWM4
        __I pcsr_tpwm_counter2_t TPWM_COUNTER2_bf; // Bit access for TPWM_COUNTER2 register
    };
} pcsr_t;

#define PCSR ((pcsr_t*)(PCSR_BASE_ADDR))

#ifdef __cplusplus
}
#endif

#endif /* __REGS_PWM_H */