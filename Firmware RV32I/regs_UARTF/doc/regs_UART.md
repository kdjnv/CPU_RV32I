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

Base address: 0x60000000

| Name                     | Address    | Description |
| :---                     | :---       | :---        |
| [U_CTRL](#u_ctrl)        | 0x00000000 | UART Configuration Register |
| [U_STAT](#u_stat)        | 0x00000004 | UART Status Register |
| [U_TXDATA](#u_txdata)    | 0x00000008 | UART Transmit Data Register |
| [U_RXDATA](#u_rxdata)    | 0x0000000c | UART Receive Data Register |

## U_CTRL

UART Configuration Register

Address offset: 0x00000000

Reset value: 0x000000f0

![u_ctrl](md_img/u_ctrl.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:16  | -               | 0x0000     | Reserved |
| CLK              | 15:8   | rw              | 0x00       | System Clock Frequency (in MHz or other unit, implementation-defined) |
| BR               | 7:4    | rw              | 0xf        | Baud Rate Selector |
| -                | 3:2    | -               | 0x0        | Reserved |
| STRTX            | 1      | rw              | 0x0        | Start Transmission (1 cycle pulse) |
| EN               | 0      | rw              | 0x0        | Enable UART |

Enumerated values for U_CTRL.BR.

| Name             | Value   | Description |
| :---             | :---    | :---        |
| BR_600           | 0x0    | 600 bps |
| BR_1200          | 0x1    | 1200 bps |
| BR_2400          | 0x2    | 2400 bps |
| BR_4800          | 0x3    | 4800 bps |
| BR_9600          | 0x4    | 9600 bps |
| BR_14400         | 0x5    | 14400 bps |
| BR_19200         | 0x6    | 19200 bps |
| BR_38400         | 0x7    | 38400 bps |
| BR_56000         | 0x8    | 56000 bps |
| BR_57600         | 0x9    | 57600 bps |
| BR_115200        | 0xf    | 115200 bps |

Back to [Register map](#register-map-summary).

## U_STAT

UART Status Register

Address offset: 0x00000004

Reset value: 0x00000000

![u_stat](md_img/u_stat.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:2   | -               | 0x0000000  | Reserved |
| RXNE             | 1      | ro              | 0x0        | Receive Buffer Not Empty |
| TBUSY            | 0      | ro              | 0x0        | Transmitter Busy |

Back to [Register map](#register-map-summary).

## U_TXDATA

UART Transmit Data Register

Address offset: 0x00000008

Reset value: 0x00000000

![u_txdata](md_img/u_txdata.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:8   | -               | 0x000000   | Reserved |
| DATA             | 7:0    | rw              | 0x00       | Data To Transmit |

Back to [Register map](#register-map-summary).

## U_RXDATA

UART Receive Data Register

Address offset: 0x0000000c

Reset value: 0x00000000

![u_rxdata](md_img/u_rxdata.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:8   | -               | 0x000000   | Reserved |
| DATA             | 7:0    | ro              | 0x00       | Received Data |

Back to [Register map](#register-map-summary).
