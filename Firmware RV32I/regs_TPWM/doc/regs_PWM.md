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
| [TPWM_config](#tpwm_config) | 0x00000000 | PWM configuration register |
| [TPWM_prescaler](#tpwm_prescaler) | 0x00000004 | PWM clock divider |
| [TPWM_period1](#tpwm_period1) | 0x00000008 | PWM1 and PWM2 period register |
| [TPWM_period2](#tpwm_period2) | 0x0000000c | PWM3 and PWM4 period register |
| [TPWM_compare1](#tpwm_compare1) | 0x00000010 | Compare values for PWM1 and PWM2 |
| [TPWM_compare2](#tpwm_compare2) | 0x00000014 | Compare values for PWM3 and PWM4 |
| [TPWM_counter1](#tpwm_counter1) | 0x00000018 | PWM counters for PWM1 and PWM2 |
| [TPWM_counter2](#tpwm_counter2) | 0x0000001c | PWM counters for PWM3 and PWM4 |

## TPWM_config

PWM configuration register

Address offset: 0x00000000

Reset value: 0x00000000

![tpwm_config](md_img/tpwm_config.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:15  | -               | 0x0000     | Reserved |
| POL              | 14     | rw              | 0x0        | Polarity of the PWM signal |
| NUM              | 13:11  | rw              | 0x0        | Number of PWM signals selected |
| SEL4             | 10:9   | rw              | 0x0        | Output selection for PWM4 |
| SEL3             | 8:7    | rw              | 0x0        | Output selection for PWM3 |
| RESERVED1        | 6:5    | rw              | 0x0        | Reserved for future use |
| SEL2             | 4:3    | rw              | 0x0        | Output selection for PWM2 |
| SEL1             | 2:1    | rw              | 0x0        | Output selection for PWM1 |
| EN               | 0      | rw              | 0x0        | Enable PWM operation |

Back to [Register map](#register-map-summary).

## TPWM_prescaler

PWM clock divider

Address offset: 0x00000004

Reset value: 0x0000001b

![tpwm_prescaler](md_img/tpwm_prescaler.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| DIV              | 31:0   | rw              | 0x0000001b | Clock divider value for PWM |

Back to [Register map](#register-map-summary).

## TPWM_period1

PWM1 and PWM2 period register

Address offset: 0x00000008

Reset value: 0xffffffff

![tpwm_period1](md_img/tpwm_period1.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| PER2             | 31:16  | rw              | 0xffff     | Period value for PWM2 |
| PER1             | 15:0   | rw              | 0xffff     | Period value for PWM1 |

Back to [Register map](#register-map-summary).

## TPWM_period2

PWM3 and PWM4 period register

Address offset: 0x0000000c

Reset value: 0xffffffff

![tpwm_period2](md_img/tpwm_period2.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| PER4             | 31:16  | rw              | 0xffff     | Period value for PWM4 |
| PER3             | 15:0   | rw              | 0xffff     | Period value for PWM3 |

Back to [Register map](#register-map-summary).

## TPWM_compare1

Compare values for PWM1 and PWM2

Address offset: 0x00000010

Reset value: 0xffffffff

![tpwm_compare1](md_img/tpwm_compare1.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| CP2              | 31:16  | rw              | 0xffff     | Compare value for PWM2 |
| CP1              | 15:0   | rw              | 0xffff     | Compare value for PWM1 |

Back to [Register map](#register-map-summary).

## TPWM_compare2

Compare values for PWM3 and PWM4

Address offset: 0x00000014

Reset value: 0xffffffff

![tpwm_compare2](md_img/tpwm_compare2.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| CP4              | 31:16  | rw              | 0xffff     | Compare value for PWM4 |
| CP3              | 15:0   | rw              | 0xffff     | Compare value for PWM3 |

Back to [Register map](#register-map-summary).

## TPWM_counter1

PWM counters for PWM1 and PWM2

Address offset: 0x00000018

Reset value: 0x00000000

![tpwm_counter1](md_img/tpwm_counter1.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| CNT2             | 31:16  | ro              | 0x0000     | Current counter value for PWM2 |
| CNT1             | 15:0   | ro              | 0x0000     | Current counter value for PWM1 |

Back to [Register map](#register-map-summary).

## TPWM_counter2

PWM counters for PWM3 and PWM4

Address offset: 0x0000001c

Reset value: 0x00000000

![tpwm_counter2](md_img/tpwm_counter2.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| CNT4             | 31:16  | ro              | 0x0000     | Current counter value for PWM4 |
| CNT3             | 15:0   | ro              | 0x0000     | Current counter value for PWM3 |

Back to [Register map](#register-map-summary).
