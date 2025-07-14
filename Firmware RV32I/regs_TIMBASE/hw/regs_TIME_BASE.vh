// Created with Corsair v1.0.4

`ifndef __REGS_TIME_BASE_VH
`define __REGS_TIME_BASE_VH

`define TMR_BASE_ADDR 0
`define TMR_DATA_WIDTH 32
`define TMR_ADDR_WIDTH 32

// Tim_config - Timer configuration register
`define TMR_TIM_CONFIG_ADDR 32'h0
`define TMR_TIM_CONFIG_RESET 32'h2

// Tim_config.EN - Enable timer
`define TMR_TIM_CONFIG_EN_WIDTH 1
`define TMR_TIM_CONFIG_EN_LSB 0
`define TMR_TIM_CONFIG_EN_MASK 32'h0
`define TMR_TIM_CONFIG_EN_RESET 1'h0

// Tim_config.AR - Auto reload when reaching period
`define TMR_TIM_CONFIG_AR_WIDTH 1
`define TMR_TIM_CONFIG_AR_LSB 1
`define TMR_TIM_CONFIG_AR_MASK 32'h0
`define TMR_TIM_CONFIG_AR_RESET 1'h1

// Tim_config.DIR - Count direction (0 = up, 1 = down)
`define TMR_TIM_CONFIG_DIR_WIDTH 1
`define TMR_TIM_CONFIG_DIR_LSB 4
`define TMR_TIM_CONFIG_DIR_MASK 32'h0
`define TMR_TIM_CONFIG_DIR_RESET 1'h0

// Tim_config.UD - Update counter with load value
`define TMR_TIM_CONFIG_UD_WIDTH 1
`define TMR_TIM_CONFIG_UD_LSB 5
`define TMR_TIM_CONFIG_UD_MASK 32'h0
`define TMR_TIM_CONFIG_UD_RESET 1'h0


// Tim_prescaler - Prescaler register (divides input clock)
`define TMR_TIM_PRESCALER_ADDR 32'h4
`define TMR_TIM_PRESCALER_RESET 32'h1b

// Tim_prescaler.DIV - Clock divider value
`define TMR_TIM_PRESCALER_DIV_WIDTH 32
`define TMR_TIM_PRESCALER_DIV_LSB 0
`define TMR_TIM_PRESCALER_DIV_MASK 32'h4
`define TMR_TIM_PRESCALER_DIV_RESET 32'h1b


// Tim_period - Timer period register
`define TMR_TIM_PERIOD_ADDR 32'h8
`define TMR_TIM_PERIOD_RESET 32'hffff

// Tim_period.PER - Counter max value
`define TMR_TIM_PERIOD_PER_WIDTH 32
`define TMR_TIM_PERIOD_PER_LSB 0
`define TMR_TIM_PERIOD_PER_MASK 32'h8
`define TMR_TIM_PERIOD_PER_RESET 32'hffff


// Tim_counter - Current counter value
`define TMR_TIM_COUNTER_ADDR 32'hc
`define TMR_TIM_COUNTER_RESET 32'h0

// Tim_counter.CNT - Counter current count
`define TMR_TIM_COUNTER_CNT_WIDTH 32
`define TMR_TIM_COUNTER_CNT_LSB 0
`define TMR_TIM_COUNTER_CNT_MASK 32'hc
`define TMR_TIM_COUNTER_CNT_RESET 32'h0


// Tim_status - Timer status register
`define TMR_TIM_STATUS_ADDR 32'h10
`define TMR_TIM_STATUS_RESET 32'h0

// Tim_status.OF - Overflow flag
`define TMR_TIM_STATUS_OF_WIDTH 1
`define TMR_TIM_STATUS_OF_LSB 0
`define TMR_TIM_STATUS_OF_MASK 32'h10
`define TMR_TIM_STATUS_OF_RESET 1'h0

// Tim_status.OP - Over period flag
`define TMR_TIM_STATUS_OP_WIDTH 1
`define TMR_TIM_STATUS_OP_LSB 1
`define TMR_TIM_STATUS_OP_MASK 32'h10
`define TMR_TIM_STATUS_OP_RESET 1'h0

// Tim_status.ERR - Error flag
`define TMR_TIM_STATUS_ERR_WIDTH 1
`define TMR_TIM_STATUS_ERR_LSB 2
`define TMR_TIM_STATUS_ERR_MASK 32'h10
`define TMR_TIM_STATUS_ERR_RESET 1'h0

// Tim_status.LD - Load done flag
`define TMR_TIM_STATUS_LD_WIDTH 1
`define TMR_TIM_STATUS_LD_LSB 3
`define TMR_TIM_STATUS_LD_MASK 32'h10
`define TMR_TIM_STATUS_LD_RESET 1'h0


// Tim_load - Value to load into counter
`define TMR_TIM_LOAD_ADDR 32'h14
`define TMR_TIM_LOAD_RESET 32'h0

// Tim_load.LOAD - Counter load value
`define TMR_TIM_LOAD_LOAD_WIDTH 32
`define TMR_TIM_LOAD_LOAD_LSB 0
`define TMR_TIM_LOAD_LOAD_MASK 32'h14
`define TMR_TIM_LOAD_LOAD_RESET 32'h0


`endif // __REGS_TIME_BASE_VH