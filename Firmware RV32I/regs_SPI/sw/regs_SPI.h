// Created with Corsair v1.0.4
#ifndef __REGS_SPI_H
#define __REGS_SPI_H

#define __I  volatile const // 'read only' permissions
#define __O  volatile       // 'write only' permissions
#define __IO volatile       // 'read / write' permissions


#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define SPI_BASE_ADDR 0x90000000

// SPI_TX_LOW - Transmit data register low (lower 32 bits)
#define SPI_SPI_TX_LOW_ADDR 0x0
#define SPI_SPI_TX_LOW_RESET 0x0
typedef struct {
    uint32_t DATA_LOW : 32; // Data to transmit (lower 32 bits)
} spi_spi_tx_low_t;

// SPI_TX_LOW.DATA_LOW - Data to transmit (lower 32 bits)
#define SPI_SPI_TX_LOW_DATA_LOW_WIDTH 32
#define SPI_SPI_TX_LOW_DATA_LOW_LSB 0
#define SPI_SPI_TX_LOW_DATA_LOW_MASK 0xffffffff
#define SPI_SPI_TX_LOW_DATA_LOW_RESET 0x0

// SPI_TX_HIGH - Transmit data register high (higher 32 bits)
#define SPI_SPI_TX_HIGH_ADDR 0x4
#define SPI_SPI_TX_HIGH_RESET 0x0
typedef struct {
    uint32_t DATA_HIGH : 32; // Data to transmit (higher 32 bits)
} spi_spi_tx_high_t;

// SPI_TX_HIGH.DATA_HIGH - Data to transmit (higher 32 bits)
#define SPI_SPI_TX_HIGH_DATA_HIGH_WIDTH 32
#define SPI_SPI_TX_HIGH_DATA_HIGH_LSB 0
#define SPI_SPI_TX_HIGH_DATA_HIGH_MASK 0xffffffff
#define SPI_SPI_TX_HIGH_DATA_HIGH_RESET 0x0

// SPI_RX_LOW - Receive data register low (lower 32 bits)
#define SPI_SPI_RX_LOW_ADDR 0x8
#define SPI_SPI_RX_LOW_RESET 0x0
typedef struct {
    uint32_t DATA : 32; // Data received (lower 32 bits)
} spi_spi_rx_low_t;

// SPI_RX_LOW.DATA - Data received (lower 32 bits)
#define SPI_SPI_RX_LOW_DATA_WIDTH 32
#define SPI_SPI_RX_LOW_DATA_LSB 0
#define SPI_SPI_RX_LOW_DATA_MASK 0xffffffff
#define SPI_SPI_RX_LOW_DATA_RESET 0x0

// SPI_RX_HIGH - Receive data register high (higher 32 bits)
#define SPI_SPI_RX_HIGH_ADDR 0xc
#define SPI_SPI_RX_HIGH_RESET 0x0
typedef struct {
    uint32_t DATA : 32; // Data received (higher 32 bits)
} spi_spi_rx_high_t;

// SPI_RX_HIGH.DATA - Data received (higher 32 bits)
#define SPI_SPI_RX_HIGH_DATA_WIDTH 32
#define SPI_SPI_RX_HIGH_DATA_LSB 0
#define SPI_SPI_RX_HIGH_DATA_MASK 0xffffffff
#define SPI_SPI_RX_HIGH_DATA_RESET 0x0

// CONTROL_REG - SPI Control register
#define SPI_CONTROL_REG_ADDR 0x10
#define SPI_CONTROL_REG_RESET 0x6d0208
typedef struct {
    uint32_t BPT : 6; // Bits per transfer (0 -> 1 bit, 63 -> 64 bits)
    uint32_t CPOL : 1; // Clock polarity
    uint32_t CPHA : 1; // Clock phase
    uint32_t ENSPI : 1; // Enable SPI
    uint32_t CSS : 1; // Automatic slave select enable
    uint32_t : 1; // reserved
    uint32_t NUMSS : 4; // Slave number selection (0~15)
    uint32_t : 1; // reserved
    uint32_t MSB : 1; // MSB first enable
    uint32_t STRX : 1; // Start transmit/receive
    uint32_t DIV : 10; // Clock divider
    uint32_t MODE : 1; // 0 - Master mode, 1 - Slave mode
    uint32_t : 3; // reserved
} spi_control_reg_t;

// CONTROL_REG.BPT - Bits per transfer (0 -> 1 bit, 63 -> 64 bits)
#define SPI_CONTROL_REG_BPT_WIDTH 6
#define SPI_CONTROL_REG_BPT_LSB 0
#define SPI_CONTROL_REG_BPT_MASK 0x3f
#define SPI_CONTROL_REG_BPT_RESET 0x8

// CONTROL_REG.CPOL - Clock polarity
#define SPI_CONTROL_REG_CPOL_WIDTH 1
#define SPI_CONTROL_REG_CPOL_LSB 6
#define SPI_CONTROL_REG_CPOL_MASK 0x40
#define SPI_CONTROL_REG_CPOL_RESET 0x0

// CONTROL_REG.CPHA - Clock phase
#define SPI_CONTROL_REG_CPHA_WIDTH 1
#define SPI_CONTROL_REG_CPHA_LSB 7
#define SPI_CONTROL_REG_CPHA_MASK 0x80
#define SPI_CONTROL_REG_CPHA_RESET 0x0

// CONTROL_REG.ENSPI - Enable SPI
#define SPI_CONTROL_REG_ENSPI_WIDTH 1
#define SPI_CONTROL_REG_ENSPI_LSB 8
#define SPI_CONTROL_REG_ENSPI_MASK 0x100
#define SPI_CONTROL_REG_ENSPI_RESET 0x0

// CONTROL_REG.CSS - Automatic slave select enable
#define SPI_CONTROL_REG_CSS_WIDTH 1
#define SPI_CONTROL_REG_CSS_LSB 9
#define SPI_CONTROL_REG_CSS_MASK 0x200
#define SPI_CONTROL_REG_CSS_RESET 0x1

// CONTROL_REG.NUMSS - Slave number selection (0~15)
#define SPI_CONTROL_REG_NUMSS_WIDTH 4
#define SPI_CONTROL_REG_NUMSS_LSB 11
#define SPI_CONTROL_REG_NUMSS_MASK 0x7800
#define SPI_CONTROL_REG_NUMSS_RESET 0x0

// CONTROL_REG.MSB - MSB first enable
#define SPI_CONTROL_REG_MSB_WIDTH 1
#define SPI_CONTROL_REG_MSB_LSB 16
#define SPI_CONTROL_REG_MSB_MASK 0x10000
#define SPI_CONTROL_REG_MSB_RESET 0x1

// CONTROL_REG.STRX - Start transmit/receive
#define SPI_CONTROL_REG_STRX_WIDTH 1
#define SPI_CONTROL_REG_STRX_LSB 17
#define SPI_CONTROL_REG_STRX_MASK 0x20000
#define SPI_CONTROL_REG_STRX_RESET 0x0

// CONTROL_REG.DIV - Clock divider
#define SPI_CONTROL_REG_DIV_WIDTH 10
#define SPI_CONTROL_REG_DIV_LSB 18
#define SPI_CONTROL_REG_DIV_MASK 0xffc0000
#define SPI_CONTROL_REG_DIV_RESET 0x1b

// CONTROL_REG.MODE - 0 - Master mode, 1 - Slave mode
#define SPI_CONTROL_REG_MODE_WIDTH 1
#define SPI_CONTROL_REG_MODE_LSB 28
#define SPI_CONTROL_REG_MODE_MASK 0x10000000
#define SPI_CONTROL_REG_MODE_RESET 0x0

// STATUS_REG - SPI Status register
#define SPI_STATUS_REG_ADDR 0x14
#define SPI_STATUS_REG_RESET 0x0
typedef struct {
    uint32_t RXNE : 1; // RX not empty (data received)
    uint32_t TXE : 1; // TX empty (ready for transmit)
    uint32_t ERR : 1; // Error flag (future use)
    uint32_t : 1; // reserved
    uint32_t BUSY : 1; // SPI Busy flag
    uint32_t : 27; // reserved
} spi_status_reg_t;

// STATUS_REG.RXNE - RX not empty (data received)
#define SPI_STATUS_REG_RXNE_WIDTH 1
#define SPI_STATUS_REG_RXNE_LSB 0
#define SPI_STATUS_REG_RXNE_MASK 0x1
#define SPI_STATUS_REG_RXNE_RESET 0x0

// STATUS_REG.TXE - TX empty (ready for transmit)
#define SPI_STATUS_REG_TXE_WIDTH 1
#define SPI_STATUS_REG_TXE_LSB 1
#define SPI_STATUS_REG_TXE_MASK 0x2
#define SPI_STATUS_REG_TXE_RESET 0x0

// STATUS_REG.ERR - Error flag (future use)
#define SPI_STATUS_REG_ERR_WIDTH 1
#define SPI_STATUS_REG_ERR_LSB 2
#define SPI_STATUS_REG_ERR_MASK 0x4
#define SPI_STATUS_REG_ERR_RESET 0x0

// STATUS_REG.BUSY - SPI Busy flag
#define SPI_STATUS_REG_BUSY_WIDTH 1
#define SPI_STATUS_REG_BUSY_LSB 4
#define SPI_STATUS_REG_BUSY_MASK 0x10
#define SPI_STATUS_REG_BUSY_RESET 0x0


// Register map structure
typedef struct {
    union {
        __IO uint32_t SPI_TX_LOW; // Transmit data register low (lower 32 bits)
        __IO spi_spi_tx_low_t SPI_TX_LOW_bf; // Bit access for SPI_TX_LOW register
    };
    union {
        __IO uint32_t SPI_TX_HIGH; // Transmit data register high (higher 32 bits)
        __IO spi_spi_tx_high_t SPI_TX_HIGH_bf; // Bit access for SPI_TX_HIGH register
    };
    union {
        __IO uint32_t SPI_RX_LOW; // Receive data register low (lower 32 bits)
        __IO spi_spi_rx_low_t SPI_RX_LOW_bf; // Bit access for SPI_RX_LOW register
    };
    union {
        __I uint32_t SPI_RX_HIGH; // Receive data register high (higher 32 bits)
        __I spi_spi_rx_high_t SPI_RX_HIGH_bf; // Bit access for SPI_RX_HIGH register
    };
    union {
        __IO uint32_t CONTROL_REG; // SPI Control register
        __IO spi_control_reg_t CONTROL_REG_bf; // Bit access for CONTROL_REG register
    };
    union {
        __I uint32_t STATUS_REG; // SPI Status register
        __I spi_status_reg_t STATUS_REG_bf; // Bit access for STATUS_REG register
    };
} spi_t;

#define SPI ((spi_t*)(SPI_BASE_ADDR))

#ifdef __cplusplus
}
#endif

#endif /* __REGS_SPI_H */