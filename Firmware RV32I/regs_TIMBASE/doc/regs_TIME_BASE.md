# TIME_BASE Register Map

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
| [Tim_config](#tim_config) | 0x00000000 | Timer configuration register |
| [Tim_prescaler](#tim_prescaler) | 0x00000004 | Prescaler register (divides input clock) |
| [Tim_period](#tim_period) | 0x00000008 | Timer period register |
| [Tim_counter](#tim_counter) | 0x0000000c | Current counter value |
| [Tim_status](#tim_status) | 0x00000010 | Timer status register |
| [Tim_load](#tim_load)    | 0x00000014 | Value to load into counter |

## Tim_config

Timer configuration register

Address offset: 0x00000000

Reset value: 0x00000002

![tim_config](md_img/tim_config.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:6   | -               | 0x000000   | Reserved |
| UD               | 5      | wo              | 0x0        | Update counter with load value |
| DIR              | 4      | wo              | 0x0        | Count direction (0 = up, 1 = down) |
| -                | 3:2    | -               | 0x0        | Reserved |
| AR               | 1      | wo              | 0x1        | Auto reload when reaching period |
| EN               | 0      | wo              | 0x0        | Enable timer |

Back to [Register map](#register-map-summary).

## Tim_prescaler

Prescaler register (divides input clock)

Address offset: 0x00000004

Reset value: 0x0000001b

![tim_prescaler](md_img/tim_prescaler.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| DIV              | 31:0   | wo              | 0x0000001b | Clock divider value |

Back to [Register map](#register-map-summary).

## Tim_period

Timer period register

Address offset: 0x00000008

Reset value: 0x0000ffff

![tim_period](md_img/tim_period.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| PER              | 31:0   | wo              | 0x0000ffff | Counter max value |

Back to [Register map](#register-map-summary).

## Tim_counter

Current counter value

Address offset: 0x0000000c

Reset value: 0x00000000

![tim_counter](md_img/tim_counter.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| CNT              | 31:0   | ro              | 0x00000000 | Counter current count |

Back to [Register map](#register-map-summary).

## Tim_status

Timer status register

Address offset: 0x00000010

Reset value: 0x00000000

![tim_status](md_img/tim_status.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:4   | -               | 0x0000000  | Reserved |
| LD               | 3      | ro              | 0x0        | Load done flag |
| ERR              | 2      | ro              | 0x0        | Error flag |
| OP               | 1      | ro              | 0x0        | Over period flag |
| OF               | 0      | ro              | 0x0        | Overflow flag |

Back to [Register map](#register-map-summary).

## Tim_load

Value to load into counter

Address offset: 0x00000014

Reset value: 0x00000000

![tim_load](md_img/tim_load.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| LOAD             | 31:0   | wo              | 0x00000000 | Counter load value |

Back to [Register map](#register-map-summary).
