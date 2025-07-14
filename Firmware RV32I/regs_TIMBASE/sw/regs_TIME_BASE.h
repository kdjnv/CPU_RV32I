// Created with Corsair v1.0.4
#ifndef __REGS_TIME_BASE_H
#define __REGS_TIME_BASE_H

#define __I  volatile const // 'read only' permissions
#define __O  volatile       // 'write only' permissions
#define __IO volatile       // 'read / write' permissions


#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define TMR_BASE_ADDR 0x0

// Tim_config - Timer configuration register
#define TMR_TIM_CONFIG_ADDR 0x0
#define TMR_TIM_CONFIG_RESET 0x2
typedef struct {
    uint32_t EN : 1; // Enable timer
    uint32_t AR : 1; // Auto reload when reaching period
    uint32_t : 2; // reserved
    uint32_t DIR : 1; // Count direction (0 = up, 1 = down)
    uint32_t UD : 1; // Update counter with load value
    uint32_t : 26; // reserved
} tmr_tim_config_t;

// Tim_config.EN - Enable timer
#define TMR_TIM_CONFIG_EN_WIDTH 1
#define TMR_TIM_CONFIG_EN_LSB 0
#define TMR_TIM_CONFIG_EN_MASK 0x1
#define TMR_TIM_CONFIG_EN_RESET 0x0

// Tim_config.AR - Auto reload when reaching period
#define TMR_TIM_CONFIG_AR_WIDTH 1
#define TMR_TIM_CONFIG_AR_LSB 1
#define TMR_TIM_CONFIG_AR_MASK 0x2
#define TMR_TIM_CONFIG_AR_RESET 0x1

// Tim_config.DIR - Count direction (0 = up, 1 = down)
#define TMR_TIM_CONFIG_DIR_WIDTH 1
#define TMR_TIM_CONFIG_DIR_LSB 4
#define TMR_TIM_CONFIG_DIR_MASK 0x10
#define TMR_TIM_CONFIG_DIR_RESET 0x0

// Tim_config.UD - Update counter with load value
#define TMR_TIM_CONFIG_UD_WIDTH 1
#define TMR_TIM_CONFIG_UD_LSB 5
#define TMR_TIM_CONFIG_UD_MASK 0x20
#define TMR_TIM_CONFIG_UD_RESET 0x0

// Tim_prescaler - Prescaler register (divides input clock)
#define TMR_TIM_PRESCALER_ADDR 0x4
#define TMR_TIM_PRESCALER_RESET 0x1b
typedef struct {
    uint32_t DIV : 32; // Clock divider value
} tmr_tim_prescaler_t;

// Tim_prescaler.DIV - Clock divider value
#define TMR_TIM_PRESCALER_DIV_WIDTH 32
#define TMR_TIM_PRESCALER_DIV_LSB 0
#define TMR_TIM_PRESCALER_DIV_MASK 0xffffffff
#define TMR_TIM_PRESCALER_DIV_RESET 0x1b

// Tim_period - Timer period register
#define TMR_TIM_PERIOD_ADDR 0x8
#define TMR_TIM_PERIOD_RESET 0xffff
typedef struct {
    uint32_t PER : 32; // Counter max value
} tmr_tim_period_t;

// Tim_period.PER - Counter max value
#define TMR_TIM_PERIOD_PER_WIDTH 32
#define TMR_TIM_PERIOD_PER_LSB 0
#define TMR_TIM_PERIOD_PER_MASK 0xffffffff
#define TMR_TIM_PERIOD_PER_RESET 0xffff

// Tim_counter - Current counter value
#define TMR_TIM_COUNTER_ADDR 0xc
#define TMR_TIM_COUNTER_RESET 0x0
typedef struct {
    uint32_t CNT : 32; // Counter current count
} tmr_tim_counter_t;

// Tim_counter.CNT - Counter current count
#define TMR_TIM_COUNTER_CNT_WIDTH 32
#define TMR_TIM_COUNTER_CNT_LSB 0
#define TMR_TIM_COUNTER_CNT_MASK 0xffffffff
#define TMR_TIM_COUNTER_CNT_RESET 0x0

// Tim_status - Timer status register
#define TMR_TIM_STATUS_ADDR 0x10
#define TMR_TIM_STATUS_RESET 0x0
typedef struct {
    uint32_t OF : 1; // Overflow flag
    uint32_t OP : 1; // Over period flag
    uint32_t ERR : 1; // Error flag
    uint32_t LD : 1; // Load done flag
    uint32_t : 28; // reserved
} tmr_tim_status_t;

// Tim_status.OF - Overflow flag
#define TMR_TIM_STATUS_OF_WIDTH 1
#define TMR_TIM_STATUS_OF_LSB 0
#define TMR_TIM_STATUS_OF_MASK 0x1
#define TMR_TIM_STATUS_OF_RESET 0x0

// Tim_status.OP - Over period flag
#define TMR_TIM_STATUS_OP_WIDTH 1
#define TMR_TIM_STATUS_OP_LSB 1
#define TMR_TIM_STATUS_OP_MASK 0x2
#define TMR_TIM_STATUS_OP_RESET 0x0

// Tim_status.ERR - Error flag
#define TMR_TIM_STATUS_ERR_WIDTH 1
#define TMR_TIM_STATUS_ERR_LSB 2
#define TMR_TIM_STATUS_ERR_MASK 0x4
#define TMR_TIM_STATUS_ERR_RESET 0x0

// Tim_status.LD - Load done flag
#define TMR_TIM_STATUS_LD_WIDTH 1
#define TMR_TIM_STATUS_LD_LSB 3
#define TMR_TIM_STATUS_LD_MASK 0x8
#define TMR_TIM_STATUS_LD_RESET 0x0

// Tim_load - Value to load into counter
#define TMR_TIM_LOAD_ADDR 0x14
#define TMR_TIM_LOAD_RESET 0x0
typedef struct {
    uint32_t LOAD : 32; // Counter load value
} tmr_tim_load_t;

// Tim_load.LOAD - Counter load value
#define TMR_TIM_LOAD_LOAD_WIDTH 32
#define TMR_TIM_LOAD_LOAD_LSB 0
#define TMR_TIM_LOAD_LOAD_MASK 0xffffffff
#define TMR_TIM_LOAD_LOAD_RESET 0x0


// Register map structure
typedef struct {
    union {
        __O uint32_t TIM_CONFIG; // Timer configuration register
        __O tmr_tim_config_t TIM_CONFIG_bf; // Bit access for TIM_CONFIG register
    };
    union {
        __O uint32_t TIM_PRESCALER; // Prescaler register (divides input clock)
        __O tmr_tim_prescaler_t TIM_PRESCALER_bf; // Bit access for TIM_PRESCALER register
    };
    union {
        __O uint32_t TIM_PERIOD; // Timer period register
        __O tmr_tim_period_t TIM_PERIOD_bf; // Bit access for TIM_PERIOD register
    };
    union {
        __I uint32_t TIM_COUNTER; // Current counter value
        __I tmr_tim_counter_t TIM_COUNTER_bf; // Bit access for TIM_COUNTER register
    };
    union {
        __I uint32_t TIM_STATUS; // Timer status register
        __I tmr_tim_status_t TIM_STATUS_bf; // Bit access for TIM_STATUS register
    };
    union {
        __O uint32_t TIM_LOAD; // Value to load into counter
        __O tmr_tim_load_t TIM_LOAD_bf; // Bit access for TIM_LOAD register
    };
} tmr_t;

#define TMR ((tmr_t*)(TMR_BASE_ADDR))

#ifdef __cplusplus
}
#endif

#endif /* __REGS_TIME_BASE_H */