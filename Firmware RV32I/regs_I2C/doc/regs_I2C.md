# Register map

Created with [Corsair](https://github.com/esynr3z/corsair) v1.0.4.

## Conventions

| Access mode | Description               |
| :---------- | :------------------------ |
| rw          | Read and Write            |
| rw1c        | Read and Write 1 to Clear |
| rw1s        | Read and Write 1 to Set   |
| ro          | Read Only                 |
| roc         | Read Only to Clear        |
| roll        | Read Only / Latch Low     |
| rolh        | Read Only / Latch High    |
| wo          | Write only                |
| wosc        | Write Only / Self Clear   |

## Register map summary

Base address: 0x00000000

| Name                     | Address    | Description |
| :---                     | :---       | :---        |
| [I2C_CONTROL](#i2c_control) | 0x00000000 | I2C Control Register |
| [I2C_STATUS](#i2c_status) | 0x00000004 | I2C Status Register |
| [I2C_SLAVE_ADDR](#i2c_slave_addr) | 0x00000008 | I2C Slave Register Address |
| [I2C_REG_ADDR](#i2c_reg_addr) | 0x0000000c | I2C Slave Device Address |
| [I2C_TX_DATA_LOW](#i2c_tx_data_low) | 0x00000010 | I2C Transmit Data Low 32-bit |
| [I2C_TX_DATA_HIGH](#i2c_tx_data_high) | 0x00000014 | I2C Transmit Data High 32-bit |
| [I2C_RX_DATA_LOW](#i2c_rx_data_low) | 0x00000018 | I2C Received Data Low 32-bit |
| [I2C_RX_DATA_HIGH](#i2c_rx_data_high) | 0x0000001c | I2C Received Data High 32-bit |

## I2C_CONTROL

I2C Control Register

Address offset: 0x00000000

Reset value: 0x2880a50e

![i2c_control](md_img/i2c_control.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| SEML             | 31:27  | rw              | 0x5        | Sample location on SCL pulse (max 9) |
| RST              | 26     | rw              | 0x0        | Repeat start condition |
| NUMBTX           | 25:23  | rw              | 0x1        | Number of bytes to transmit (0 means 1) |
| MODE             | 22     | rw              | 0x0        | Transfer mode (0 transmit, 1 receive) |
| STRX             | 21     | rw              | 0x0        | Start transmit/receive |
| EN               | 20     | rw              | 0x0        | Enable I2C |
| LENADD           | 19:18  | rw              | 0x0        | Slave address length |
| RSE              | 17     | rw              | 0x0        | Register Slave Enable |
| DUTYSCL          | 16:13  | rw              | 0x5        | Duty cycle for SCL (reserved for future use) |
| NUMBRX           | 12:10  | rw              | 0x1        | Number of bytes to receive (0 means 1) |
| CLKDIV           | 9:0    | rw              | 0x10e      | Clock divider for SCL |

Back to [Register map](#register-map-summary).

## I2C_STATUS

I2C Status Register

Address offset: 0x00000004

Reset value: 0x00000042

![i2c_status](md_img/i2c_status.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:15  | -               | 0x0000     | Reserved |
| RS               | 14     | ro              | 0x0        | Reception complete |
| TS               | 13     | ro              | 0x0        | Transmission complete |
| BUSY             | 12     | ro              | 0x0        | I2C busy status |
| ERR              | 11     | ro              | 0x0        | Error flag |
| -                | 10:8   | -               | 0x0        | Reserved |
| RXNE             | 7      | ro              | 0x0        | Receive register not empty |
| RXE              | 6      | ro              | 0x1        | Receive register empty |
| -                | 5:3    | -               | 0x0        | Reserved |
| TXNE             | 2      | ro              | 0x0        | Transmit register not empty |
| TXE              | 1      | ro              | 0x1        | Transmit register empty |
| SLAVE_FOUND      | 0      | ro              | 0x0        | Slave address detected |

Back to [Register map](#register-map-summary).

## I2C_SLAVE_ADDR

I2C Slave Register Address

Address offset: 0x00000008

Reset value: 0x00000000

![i2c_slave_addr](md_img/i2c_slave_addr.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:10  | -               | 0x00000    | Reserved |
| ADDRESS          | 9:0    | rw              | 0x00       | 8-bit register address |

Back to [Register map](#register-map-summary).

## I2C_REG_ADDR

I2C Slave Device Address

Address offset: 0x0000000c

Reset value: 0x00000000

![i2c_reg_addr](md_img/i2c_reg_addr.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:8   | -               | 0x000000   | Reserved |
| ADDS             | 7:0    | rw              | 0x00       | 7 to 10-bit slave address |

Back to [Register map](#register-map-summary).

## I2C_TX_DATA_LOW

I2C Transmit Data Low 32-bit

Address offset: 0x00000010

Reset value: 0x00000000

![i2c_tx_data_low](md_img/i2c_tx_data_low.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| TX_DATA_LOW      | 31:0   | rw              | 0x00000000 | Lower 32-bit of data to transmit |

Back to [Register map](#register-map-summary).

## I2C_TX_DATA_HIGH

I2C Transmit Data High 32-bit

Address offset: 0x00000014

Reset value: 0x00000000

![i2c_tx_data_high](md_img/i2c_tx_data_high.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| TX_DATA_HIGH     | 31:0   | rw              | 0x00000000 | Upper 32-bit of data to transmit |

Back to [Register map](#register-map-summary).

## I2C_RX_DATA_LOW

I2C Received Data Low 32-bit

Address offset: 0x00000018

Reset value: 0x00000000

![i2c_rx_data_low](md_img/i2c_rx_data_low.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| RX_DATA_LOW      | 31:0   | ro              | 0x00000000 | Lower 32-bit of received data |

Back to [Register map](#register-map-summary).

## I2C_RX_DATA_HIGH

I2C Received Data High 32-bit

Address offset: 0x0000001c

Reset value: 0x00000000

![i2c_rx_data_high](md_img/i2c_rx_data_high.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| RX_DATA_HIGH     | 31:0   | ro              | 0x00000000 | Upper 32-bit of received data |

Back to [Register map](#register-map-summary).
