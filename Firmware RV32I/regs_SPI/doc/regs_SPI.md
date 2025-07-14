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

Base address: 0x90000000

| Name                     | Address    | Description |
| :---                     | :---       | :---        |
| [SPI_TX_LOW](#spi_tx_low) | 0x00000000 | Transmit data register low (lower 32 bits) |
| [SPI_TX_HIGH](#spi_tx_high) | 0x00000004 | Transmit data register high (higher 32 bits) |
| [SPI_RX_LOW](#spi_rx_low) | 0x00000008 | Receive data register low (lower 32 bits) |
| [SPI_RX_HIGH](#spi_rx_high) | 0x0000000c | Receive data register high (higher 32 bits) |
| [CONTROL_REG](#control_reg) | 0x00000010 | SPI Control register |
| [STATUS_REG](#status_reg) | 0x00000014 | SPI Status register |

## SPI_TX_LOW

Transmit data register low (lower 32 bits)

Address offset: 0x00000000

Reset value: 0x00000000

![spi_tx_low](md_img/spi_tx_low.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| DATA_LOW         | 31:0   | rw              | 0x00000000 | Data to transmit (lower 32 bits) |

Back to [Register map](#register-map-summary).

## SPI_TX_HIGH

Transmit data register high (higher 32 bits)

Address offset: 0x00000004

Reset value: 0x00000000

![spi_tx_high](md_img/spi_tx_high.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| DATA_HIGH        | 31:0   | rw              | 0x00000000 | Data to transmit (higher 32 bits) |

Back to [Register map](#register-map-summary).

## SPI_RX_LOW

Receive data register low (lower 32 bits)

Address offset: 0x00000008

Reset value: 0x00000000

![spi_rx_low](md_img/spi_rx_low.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| DATA             | 31:0   | rw              | 0x00000000 | Data received (lower 32 bits) |

Back to [Register map](#register-map-summary).

## SPI_RX_HIGH

Receive data register high (higher 32 bits)

Address offset: 0x0000000c

Reset value: 0x00000000

![spi_rx_high](md_img/spi_rx_high.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| DATA             | 31:0   | ro              | 0x00000000 | Data received (higher 32 bits) |

Back to [Register map](#register-map-summary).

## CONTROL_REG

SPI Control register

Address offset: 0x00000010

Reset value: 0x006d0208

![control_reg](md_img/control_reg.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:29  | -               | 0x0        | Reserved |
| MODE             | 28     | rw              | 0x0        | 0 - Master mode, 1 - Slave mode |
| DIV              | 27:18  | rw              | 0x1b       | Clock divider |
| STRX             | 17     | rw              | 0x0        | Start transmit/receive |
| MSB              | 16     | rw              | 0x1        | MSB first enable |
| -                | 15     | -               | 0x0        | Reserved |
| NUMSS            | 14:11  | rw              | 0x0        | Slave number selection (0~15) |
| -                | 10     | -               | 0x0        | Reserved |
| CSS              | 9      | rw              | 0x1        | Automatic slave select enable |
| ENSPI            | 8      | rw              | 0x0        | Enable SPI |
| CPHA             | 7      | rw              | 0x0        | Clock phase |
| CPOL             | 6      | rw              | 0x0        | Clock polarity |
| BPT              | 5:0    | rw              | 0x8        | Bits per transfer (0 -> 1 bit, 63 -> 64 bits) |

Back to [Register map](#register-map-summary).

## STATUS_REG

SPI Status register

Address offset: 0x00000014

Reset value: 0x00000000

![status_reg](md_img/status_reg.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:5   | -               | 0x000000   | Reserved |
| BUSY             | 4      | ro              | 0x0        | SPI Busy flag |
| -                | 3      | -               | 0x0        | Reserved |
| ERR              | 2      | ro              | 0x0        | Error flag (future use) |
| TXE              | 1      | ro              | 0x0        | TX empty (ready for transmit) |
| RXNE             | 0      | ro              | 0x0        | RX not empty (data received) |

Back to [Register map](#register-map-summary).
