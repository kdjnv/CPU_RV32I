// Created with Corsair v1.0.4
#ifndef __REGS_I2C_H
#define __REGS_I2C_H

#define __I  volatile const // 'read only' permissions
#define __O  volatile       // 'write only' permissions
#define __IO volatile       // 'read / write' permissions


#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

#define I2C_BASE_ADDR 0x50000000

// I2C_CONTROL - I2C Control Register
#define I2C_I2C_CONTROL_ADDR 0x0
#define I2C_I2C_CONTROL_RESET 0x2880a50e
typedef struct {
    uint32_t CLKDIV : 10; // Clock divider for SCL
    uint32_t NUMBRX : 3; // Number of bytes to receive (0 means 1)
    uint32_t DUTYSCL : 4; // Duty cycle for SCL (reserved for future use)
    uint32_t RSE : 1; // Register Slave Enable
    uint32_t LENADD : 2; // Slave address length
    uint32_t EN : 1; // Enable I2C
    uint32_t STRX : 1; // Start transmit/receive
    uint32_t MODE : 1; // Transfer mode (0 transmit, 1 receive)
    uint32_t NUMBTX : 3; // Number of bytes to transmit (0 means 1)
    uint32_t RST : 1; // Repeat start condition
    uint32_t SEML : 5; // Sample location on SCL pulse (max 9)
} i2c_i2c_control_t;

// I2C_CONTROL.CLKDIV - Clock divider for SCL
#define I2C_I2C_CONTROL_CLKDIV_WIDTH 10
#define I2C_I2C_CONTROL_CLKDIV_LSB 0
#define I2C_I2C_CONTROL_CLKDIV_MASK 0x3ff
#define I2C_I2C_CONTROL_CLKDIV_RESET 0x10e

// I2C_CONTROL.NUMBRX - Number of bytes to receive (0 means 1)
#define I2C_I2C_CONTROL_NUMBRX_WIDTH 3
#define I2C_I2C_CONTROL_NUMBRX_LSB 10
#define I2C_I2C_CONTROL_NUMBRX_MASK 0x1c00
#define I2C_I2C_CONTROL_NUMBRX_RESET 0x1

// I2C_CONTROL.DUTYSCL - Duty cycle for SCL (reserved for future use)
#define I2C_I2C_CONTROL_DUTYSCL_WIDTH 4
#define I2C_I2C_CONTROL_DUTYSCL_LSB 13
#define I2C_I2C_CONTROL_DUTYSCL_MASK 0x1e000
#define I2C_I2C_CONTROL_DUTYSCL_RESET 0x5

// I2C_CONTROL.RSE - Register Slave Enable
#define I2C_I2C_CONTROL_RSE_WIDTH 1
#define I2C_I2C_CONTROL_RSE_LSB 17
#define I2C_I2C_CONTROL_RSE_MASK 0x20000
#define I2C_I2C_CONTROL_RSE_RESET 0x0

// I2C_CONTROL.LENADD - Slave address length
#define I2C_I2C_CONTROL_LENADD_WIDTH 2
#define I2C_I2C_CONTROL_LENADD_LSB 18
#define I2C_I2C_CONTROL_LENADD_MASK 0xc0000
#define I2C_I2C_CONTROL_LENADD_RESET 0x0

// I2C_CONTROL.EN - Enable I2C
#define I2C_I2C_CONTROL_EN_WIDTH 1
#define I2C_I2C_CONTROL_EN_LSB 20
#define I2C_I2C_CONTROL_EN_MASK 0x100000
#define I2C_I2C_CONTROL_EN_RESET 0x0

// I2C_CONTROL.STRX - Start transmit/receive
#define I2C_I2C_CONTROL_STRX_WIDTH 1
#define I2C_I2C_CONTROL_STRX_LSB 21
#define I2C_I2C_CONTROL_STRX_MASK 0x200000
#define I2C_I2C_CONTROL_STRX_RESET 0x0

// I2C_CONTROL.MODE - Transfer mode (0 transmit, 1 receive)
#define I2C_I2C_CONTROL_MODE_WIDTH 1
#define I2C_I2C_CONTROL_MODE_LSB 22
#define I2C_I2C_CONTROL_MODE_MASK 0x400000
#define I2C_I2C_CONTROL_MODE_RESET 0x0

// I2C_CONTROL.NUMBTX - Number of bytes to transmit (0 means 1)
#define I2C_I2C_CONTROL_NUMBTX_WIDTH 3
#define I2C_I2C_CONTROL_NUMBTX_LSB 23
#define I2C_I2C_CONTROL_NUMBTX_MASK 0x3800000
#define I2C_I2C_CONTROL_NUMBTX_RESET 0x1

// I2C_CONTROL.RST - Repeat start condition
#define I2C_I2C_CONTROL_RST_WIDTH 1
#define I2C_I2C_CONTROL_RST_LSB 26
#define I2C_I2C_CONTROL_RST_MASK 0x4000000
#define I2C_I2C_CONTROL_RST_RESET 0x0

// I2C_CONTROL.SEML - Sample location on SCL pulse (max 9)
#define I2C_I2C_CONTROL_SEML_WIDTH 5
#define I2C_I2C_CONTROL_SEML_LSB 27
#define I2C_I2C_CONTROL_SEML_MASK 0xf8000000
#define I2C_I2C_CONTROL_SEML_RESET 0x5

// I2C_STATUS - I2C Status Register
#define I2C_I2C_STATUS_ADDR 0x4
#define I2C_I2C_STATUS_RESET 0x42
typedef struct {
    uint32_t SLAVE_FOUND : 1; // Slave address detected
    uint32_t TXE : 1; // Transmit register empty
    uint32_t TXNE : 1; // Transmit register not empty
    uint32_t : 3; // reserved
    uint32_t RXE : 1; // Receive register empty
    uint32_t RXNE : 1; // Receive register not empty
    uint32_t : 3; // reserved
    uint32_t ERR : 1; // Error flag
    uint32_t BUSY : 1; // I2C busy status
    uint32_t TS : 1; // Transmission complete
    uint32_t RS : 1; // Reception complete
    uint32_t : 17; // reserved
} i2c_i2c_status_t;

// I2C_STATUS.SLAVE_FOUND - Slave address detected
#define I2C_I2C_STATUS_SLAVE_FOUND_WIDTH 1
#define I2C_I2C_STATUS_SLAVE_FOUND_LSB 0
#define I2C_I2C_STATUS_SLAVE_FOUND_MASK 0x1
#define I2C_I2C_STATUS_SLAVE_FOUND_RESET 0x0

// I2C_STATUS.TXE - Transmit register empty
#define I2C_I2C_STATUS_TXE_WIDTH 1
#define I2C_I2C_STATUS_TXE_LSB 1
#define I2C_I2C_STATUS_TXE_MASK 0x2
#define I2C_I2C_STATUS_TXE_RESET 0x1

// I2C_STATUS.TXNE - Transmit register not empty
#define I2C_I2C_STATUS_TXNE_WIDTH 1
#define I2C_I2C_STATUS_TXNE_LSB 2
#define I2C_I2C_STATUS_TXNE_MASK 0x4
#define I2C_I2C_STATUS_TXNE_RESET 0x0

// I2C_STATUS.RXE - Receive register empty
#define I2C_I2C_STATUS_RXE_WIDTH 1
#define I2C_I2C_STATUS_RXE_LSB 6
#define I2C_I2C_STATUS_RXE_MASK 0x40
#define I2C_I2C_STATUS_RXE_RESET 0x1

// I2C_STATUS.RXNE - Receive register not empty
#define I2C_I2C_STATUS_RXNE_WIDTH 1
#define I2C_I2C_STATUS_RXNE_LSB 7
#define I2C_I2C_STATUS_RXNE_MASK 0x80
#define I2C_I2C_STATUS_RXNE_RESET 0x0

// I2C_STATUS.ERR - Error flag
#define I2C_I2C_STATUS_ERR_WIDTH 1
#define I2C_I2C_STATUS_ERR_LSB 11
#define I2C_I2C_STATUS_ERR_MASK 0x800
#define I2C_I2C_STATUS_ERR_RESET 0x0

// I2C_STATUS.BUSY - I2C busy status
#define I2C_I2C_STATUS_BUSY_WIDTH 1
#define I2C_I2C_STATUS_BUSY_LSB 12
#define I2C_I2C_STATUS_BUSY_MASK 0x1000
#define I2C_I2C_STATUS_BUSY_RESET 0x0

// I2C_STATUS.TS - Transmission complete
#define I2C_I2C_STATUS_TS_WIDTH 1
#define I2C_I2C_STATUS_TS_LSB 13
#define I2C_I2C_STATUS_TS_MASK 0x2000
#define I2C_I2C_STATUS_TS_RESET 0x0

// I2C_STATUS.RS - Reception complete
#define I2C_I2C_STATUS_RS_WIDTH 1
#define I2C_I2C_STATUS_RS_LSB 14
#define I2C_I2C_STATUS_RS_MASK 0x4000
#define I2C_I2C_STATUS_RS_RESET 0x0

// I2C_SLAVE_ADDR - I2C Slave Register Address
#define I2C_I2C_SLAVE_ADDR_ADDR 0x8
#define I2C_I2C_SLAVE_ADDR_RESET 0x0
typedef struct {
    uint32_t ADDRESS : 10; // 8-bit register address
    uint32_t : 22; // reserved
} i2c_i2c_slave_addr_t;

// I2C_SLAVE_ADDR.ADDRESS - 8-bit register address
#define I2C_I2C_SLAVE_ADDR_ADDRESS_WIDTH 10
#define I2C_I2C_SLAVE_ADDR_ADDRESS_LSB 0
#define I2C_I2C_SLAVE_ADDR_ADDRESS_MASK 0x3ff
#define I2C_I2C_SLAVE_ADDR_ADDRESS_RESET 0x0

// I2C_REG_ADDR - I2C Slave Device Address
#define I2C_I2C_REG_ADDR_ADDR 0xc
#define I2C_I2C_REG_ADDR_RESET 0x0
typedef struct {
    uint32_t ADDS : 8; // 7 to 10-bit slave address
    uint32_t : 24; // reserved
} i2c_i2c_reg_addr_t;

// I2C_REG_ADDR.ADDS - 7 to 10-bit slave address
#define I2C_I2C_REG_ADDR_ADDS_WIDTH 8
#define I2C_I2C_REG_ADDR_ADDS_LSB 0
#define I2C_I2C_REG_ADDR_ADDS_MASK 0xff
#define I2C_I2C_REG_ADDR_ADDS_RESET 0x0

// I2C_TX_DATA_LOW - I2C Transmit Data Low 32-bit
#define I2C_I2C_TX_DATA_LOW_ADDR 0x10
#define I2C_I2C_TX_DATA_LOW_RESET 0x0
typedef struct {
    uint32_t TX_DATA_LOW : 32; // Lower 32-bit of data to transmit
} i2c_i2c_tx_data_low_t;

// I2C_TX_DATA_LOW.TX_DATA_LOW - Lower 32-bit of data to transmit
#define I2C_I2C_TX_DATA_LOW_TX_DATA_LOW_WIDTH 32
#define I2C_I2C_TX_DATA_LOW_TX_DATA_LOW_LSB 0
#define I2C_I2C_TX_DATA_LOW_TX_DATA_LOW_MASK 0xffffffff
#define I2C_I2C_TX_DATA_LOW_TX_DATA_LOW_RESET 0x0

// I2C_TX_DATA_HIGH - I2C Transmit Data High 32-bit
#define I2C_I2C_TX_DATA_HIGH_ADDR 0x14
#define I2C_I2C_TX_DATA_HIGH_RESET 0x0
typedef struct {
    uint32_t TX_DATA_HIGH : 32; // Upper 32-bit of data to transmit
} i2c_i2c_tx_data_high_t;

// I2C_TX_DATA_HIGH.TX_DATA_HIGH - Upper 32-bit of data to transmit
#define I2C_I2C_TX_DATA_HIGH_TX_DATA_HIGH_WIDTH 32
#define I2C_I2C_TX_DATA_HIGH_TX_DATA_HIGH_LSB 0
#define I2C_I2C_TX_DATA_HIGH_TX_DATA_HIGH_MASK 0xffffffff
#define I2C_I2C_TX_DATA_HIGH_TX_DATA_HIGH_RESET 0x0

// I2C_RX_DATA_LOW - I2C Received Data Low 32-bit
#define I2C_I2C_RX_DATA_LOW_ADDR 0x18
#define I2C_I2C_RX_DATA_LOW_RESET 0x0
typedef struct {
    uint32_t RX_DATA_LOW : 32; // Lower 32-bit of received data
} i2c_i2c_rx_data_low_t;

// I2C_RX_DATA_LOW.RX_DATA_LOW - Lower 32-bit of received data
#define I2C_I2C_RX_DATA_LOW_RX_DATA_LOW_WIDTH 32
#define I2C_I2C_RX_DATA_LOW_RX_DATA_LOW_LSB 0
#define I2C_I2C_RX_DATA_LOW_RX_DATA_LOW_MASK 0xffffffff
#define I2C_I2C_RX_DATA_LOW_RX_DATA_LOW_RESET 0x0

// I2C_RX_DATA_HIGH - I2C Received Data High 32-bit
#define I2C_I2C_RX_DATA_HIGH_ADDR 0x1c
#define I2C_I2C_RX_DATA_HIGH_RESET 0x0
typedef struct {
    uint32_t RX_DATA_HIGH : 32; // Upper 32-bit of received data
} i2c_i2c_rx_data_high_t;

// I2C_RX_DATA_HIGH.RX_DATA_HIGH - Upper 32-bit of received data
#define I2C_I2C_RX_DATA_HIGH_RX_DATA_HIGH_WIDTH 32
#define I2C_I2C_RX_DATA_HIGH_RX_DATA_HIGH_LSB 0
#define I2C_I2C_RX_DATA_HIGH_RX_DATA_HIGH_MASK 0xffffffff
#define I2C_I2C_RX_DATA_HIGH_RX_DATA_HIGH_RESET 0x0


// Register map structure
typedef struct {
    union {
        __IO uint32_t I2C_CONTROL; // I2C Control Register
        __IO i2c_i2c_control_t I2C_CONTROL_bf; // Bit access for I2C_CONTROL register
    };
    union {
        __I uint32_t I2C_STATUS; // I2C Status Register
        __I i2c_i2c_status_t I2C_STATUS_bf; // Bit access for I2C_STATUS register
    };
    union {
        __IO uint32_t I2C_SLAVE_ADDR; // I2C Slave Register Address
        __IO i2c_i2c_slave_addr_t I2C_SLAVE_ADDR_bf; // Bit access for I2C_SLAVE_ADDR register
    };
    union {
        __IO uint32_t I2C_REG_ADDR; // I2C Slave Device Address
        __IO i2c_i2c_reg_addr_t I2C_REG_ADDR_bf; // Bit access for I2C_REG_ADDR register
    };
    union {
        __IO uint32_t I2C_TX_DATA_LOW; // I2C Transmit Data Low 32-bit
        __IO i2c_i2c_tx_data_low_t I2C_TX_DATA_LOW_bf; // Bit access for I2C_TX_DATA_LOW register
    };
    union {
        __IO uint32_t I2C_TX_DATA_HIGH; // I2C Transmit Data High 32-bit
        __IO i2c_i2c_tx_data_high_t I2C_TX_DATA_HIGH_bf; // Bit access for I2C_TX_DATA_HIGH register
    };
    union {
        __I uint32_t I2C_RX_DATA_LOW; // I2C Received Data Low 32-bit
        __I i2c_i2c_rx_data_low_t I2C_RX_DATA_LOW_bf; // Bit access for I2C_RX_DATA_LOW register
    };
    union {
        __I uint32_t I2C_RX_DATA_HIGH; // I2C Received Data High 32-bit
        __I i2c_i2c_rx_data_high_t I2C_RX_DATA_HIGH_bf; // Bit access for I2C_RX_DATA_HIGH register
    };
} i2c_t;

#define I2C ((i2c_t*)(I2C_BASE_ADDR))

#ifdef __cplusplus
}
#endif

#endif /* __REGS_I2C_H */