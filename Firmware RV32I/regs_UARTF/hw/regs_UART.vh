// Created with Corsair v1.0.4

`ifndef __REGS_UART_VH
`define __REGS_UART_VH

`define UART_BASE_ADDR 1610612736
`define UART_DATA_WIDTH 32
`define UART_ADDR_WIDTH 32

// U_CTRL - UART Configuration Register
`define UART_U_CTRL_ADDR 32'h0
`define UART_U_CTRL_RESET 32'hf0

// U_CTRL.EN - Enable UART
`define UART_U_CTRL_EN_WIDTH 1
`define UART_U_CTRL_EN_LSB 0
`define UART_U_CTRL_EN_MASK 32'h0
`define UART_U_CTRL_EN_RESET 1'h0

// U_CTRL.STRTX - Start Transmission (1 cycle pulse)
`define UART_U_CTRL_STRTX_WIDTH 1
`define UART_U_CTRL_STRTX_LSB 1
`define UART_U_CTRL_STRTX_MASK 32'h0
`define UART_U_CTRL_STRTX_RESET 1'h0

// U_CTRL.BR - Baud Rate Selector
`define UART_U_CTRL_BR_WIDTH 4
`define UART_U_CTRL_BR_LSB 4
`define UART_U_CTRL_BR_MASK 32'h0
`define UART_U_CTRL_BR_RESET 4'hf
`define UART_U_CTRL_BR_BR_600 4'h0 //600 bps
`define UART_U_CTRL_BR_BR_1200 4'h1 //1200 bps
`define UART_U_CTRL_BR_BR_2400 4'h2 //2400 bps
`define UART_U_CTRL_BR_BR_4800 4'h3 //4800 bps
`define UART_U_CTRL_BR_BR_9600 4'h4 //9600 bps
`define UART_U_CTRL_BR_BR_14400 4'h5 //14400 bps
`define UART_U_CTRL_BR_BR_19200 4'h6 //19200 bps
`define UART_U_CTRL_BR_BR_38400 4'h7 //38400 bps
`define UART_U_CTRL_BR_BR_56000 4'h8 //56000 bps
`define UART_U_CTRL_BR_BR_57600 4'h9 //57600 bps
`define UART_U_CTRL_BR_BR_115200 4'hf //115200 bps

// U_CTRL.CLK - System Clock Frequency (in MHz or other unit, implementation-defined)
`define UART_U_CTRL_CLK_WIDTH 8
`define UART_U_CTRL_CLK_LSB 8
`define UART_U_CTRL_CLK_MASK 32'h0
`define UART_U_CTRL_CLK_RESET 8'h0


// U_STAT - UART Status Register
`define UART_U_STAT_ADDR 32'h4
`define UART_U_STAT_RESET 32'h0

// U_STAT.TBUSY - Transmitter Busy
`define UART_U_STAT_TBUSY_WIDTH 1
`define UART_U_STAT_TBUSY_LSB 0
`define UART_U_STAT_TBUSY_MASK 32'h4
`define UART_U_STAT_TBUSY_RESET 1'h0

// U_STAT.RXNE - Receive Buffer Not Empty
`define UART_U_STAT_RXNE_WIDTH 1
`define UART_U_STAT_RXNE_LSB 1
`define UART_U_STAT_RXNE_MASK 32'h4
`define UART_U_STAT_RXNE_RESET 1'h0


// U_TXDATA - UART Transmit Data Register
`define UART_U_TXDATA_ADDR 32'h8
`define UART_U_TXDATA_RESET 32'h0

// U_TXDATA.DATA - Data To Transmit
`define UART_U_TXDATA_DATA_WIDTH 8
`define UART_U_TXDATA_DATA_LSB 0
`define UART_U_TXDATA_DATA_MASK 32'h8
`define UART_U_TXDATA_DATA_RESET 8'h0


// U_RXDATA - UART Receive Data Register
`define UART_U_RXDATA_ADDR 32'hc
`define UART_U_RXDATA_RESET 32'h0

// U_RXDATA.DATA - Received Data
`define UART_U_RXDATA_DATA_WIDTH 8
`define UART_U_RXDATA_DATA_LSB 0
`define UART_U_RXDATA_DATA_MASK 32'hc
`define UART_U_RXDATA_DATA_RESET 8'h0


`endif // __REGS_UART_VH