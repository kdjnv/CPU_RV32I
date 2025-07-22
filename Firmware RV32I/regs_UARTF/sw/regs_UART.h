// Created with Corsair v1.0.4
#ifndef __REGS_UART_H
#define __REGS_UART_H

#define __I  volatile const // 'read only' permissions
#define __O  volatile       // 'write only' permissions
#define __IO volatile       // 'read / write' permissions


#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define UART_BASE_ADDR 0x60000000

// U_CTRL - UART Configuration Register
#define UART_U_CTRL_ADDR 0x0
#define UART_U_CTRL_RESET 0xf0
typedef struct {
    uint32_t EN : 1; // Enable UART
    uint32_t STRTX : 1; // Start Transmission (1 cycle pulse)
    uint32_t : 2; // reserved
    uint32_t BR : 4; // Baud Rate Selector
    uint32_t CLK : 8; // System Clock Frequency (in MHz or other unit, implementation-defined)
    uint32_t : 16; // reserved
} uart_u_ctrl_t;

// U_CTRL.EN - Enable UART
#define UART_U_CTRL_EN_WIDTH 1
#define UART_U_CTRL_EN_LSB 0
#define UART_U_CTRL_EN_MASK 0x1
#define UART_U_CTRL_EN_RESET 0x0

// U_CTRL.STRTX - Start Transmission (1 cycle pulse)
#define UART_U_CTRL_STRTX_WIDTH 1
#define UART_U_CTRL_STRTX_LSB 1
#define UART_U_CTRL_STRTX_MASK 0x2
#define UART_U_CTRL_STRTX_RESET 0x0

// U_CTRL.BR - Baud Rate Selector
#define UART_U_CTRL_BR_WIDTH 4
#define UART_U_CTRL_BR_LSB 4
#define UART_U_CTRL_BR_MASK 0xf0
#define UART_U_CTRL_BR_RESET 0xf
typedef enum {
    UART_U_CTRL_BR_BR_600 = 0x0, //600 bps
    UART_U_CTRL_BR_BR_1200 = 0x1, //1200 bps
    UART_U_CTRL_BR_BR_2400 = 0x2, //2400 bps
    UART_U_CTRL_BR_BR_4800 = 0x3, //4800 bps
    UART_U_CTRL_BR_BR_9600 = 0x4, //9600 bps
    UART_U_CTRL_BR_BR_14400 = 0x5, //14400 bps
    UART_U_CTRL_BR_BR_19200 = 0x6, //19200 bps
    UART_U_CTRL_BR_BR_38400 = 0x7, //38400 bps
    UART_U_CTRL_BR_BR_56000 = 0x8, //56000 bps
    UART_U_CTRL_BR_BR_57600 = 0x9, //57600 bps
    UART_U_CTRL_BR_BR_115200 = 0xf, //115200 bps
} uart_u_ctrl_br_t;

// U_CTRL.CLK - System Clock Frequency (in MHz or other unit, implementation-defined)
#define UART_U_CTRL_CLK_WIDTH 8
#define UART_U_CTRL_CLK_LSB 8
#define UART_U_CTRL_CLK_MASK 0xff00
#define UART_U_CTRL_CLK_RESET 0x0

// U_STAT - UART Status Register
#define UART_U_STAT_ADDR 0x4
#define UART_U_STAT_RESET 0x0
typedef struct {
    uint32_t TBUSY : 1; // Transmitter Busy
    uint32_t RXNE : 1; // Receive Buffer Not Empty
    uint32_t : 30; // reserved
} uart_u_stat_t;

// U_STAT.TBUSY - Transmitter Busy
#define UART_U_STAT_TBUSY_WIDTH 1
#define UART_U_STAT_TBUSY_LSB 0
#define UART_U_STAT_TBUSY_MASK 0x1
#define UART_U_STAT_TBUSY_RESET 0x0

// U_STAT.RXNE - Receive Buffer Not Empty
#define UART_U_STAT_RXNE_WIDTH 1
#define UART_U_STAT_RXNE_LSB 1
#define UART_U_STAT_RXNE_MASK 0x2
#define UART_U_STAT_RXNE_RESET 0x0

// U_TXDATA - UART Transmit Data Register
#define UART_U_TXDATA_ADDR 0x8
#define UART_U_TXDATA_RESET 0x0
typedef struct {
    uint32_t DATA : 8; // Data To Transmit
    uint32_t : 24; // reserved
} uart_u_txdata_t;

// U_TXDATA.DATA - Data To Transmit
#define UART_U_TXDATA_DATA_WIDTH 8
#define UART_U_TXDATA_DATA_LSB 0
#define UART_U_TXDATA_DATA_MASK 0xff
#define UART_U_TXDATA_DATA_RESET 0x0

// U_RXDATA - UART Receive Data Register
#define UART_U_RXDATA_ADDR 0xc
#define UART_U_RXDATA_RESET 0x0
typedef struct {
    uint32_t DATA : 8; // Received Data
    uint32_t : 24; // reserved
} uart_u_rxdata_t;

// U_RXDATA.DATA - Received Data
#define UART_U_RXDATA_DATA_WIDTH 8
#define UART_U_RXDATA_DATA_LSB 0
#define UART_U_RXDATA_DATA_MASK 0xff
#define UART_U_RXDATA_DATA_RESET 0x0


// Register map structure
typedef struct {
    union {
        __IO uint32_t U_CTRL; // UART Configuration Register
        __IO uart_u_ctrl_t U_CTRL_bf; // Bit access for U_CTRL register
    };
    union {
        __I uint32_t U_STAT; // UART Status Register
        __I uart_u_stat_t U_STAT_bf; // Bit access for U_STAT register
    };
    union {
        __IO uint32_t U_TXDATA; // UART Transmit Data Register
        __IO uart_u_txdata_t U_TXDATA_bf; // Bit access for U_TXDATA register
    };
    union {
        __I uint32_t U_RXDATA; // UART Receive Data Register
        __I uart_u_rxdata_t U_RXDATA_bf; // Bit access for U_RXDATA register
    };
} uart_t;

#define UART ((uart_t*)(UART_BASE_ADDR))

#ifdef __cplusplus
}
#endif

#endif /* __REGS_UART_H */