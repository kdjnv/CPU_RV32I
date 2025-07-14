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
| [GPIO_IO](#gpio_io)      | 0x00000000 | GPIO Read/Write Register |
| [GPIO_CONFIG](#gpio_config) | 0x00000004 | GPIO_CONFIG |

## GPIO_IO

GPIO Read/Write Register

Address offset: 0x00000000

Reset value: 0x00000000

![gpio_io](md_img/gpio_io.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:16  | -               | 0x0000     | Reserved |
| GPIO_15          | 15     | rw              | 0x0        | GPIO15 |
| GPIO_14          | 14     | rw              | 0x0        | GPIO14 |
| GPIO_13          | 13     | rw              | 0x0        | GPIO13 |
| GPIO_12          | 12     | rw              | 0x0        | GPIO12 |
| GPIO_11          | 11     | rw              | 0x0        | GPIO11 |
| GPIO_10          | 10     | rw              | 0x0        | GPIO10 |
| GPIO_9           | 9      | rw              | 0x0        | GPIO9 |
| GPIO_8           | 8      | rw              | 0x0        | GPIO8 |
| GPIO_7           | 7      | rw              | 0x0        | GPIO7 |
| GPIO_6           | 6      | rw              | 0x0        | GPIO6 |
| GPIO_5           | 5      | rw              | 0x0        | GPIO5 |
| GPIO_4           | 4      | rw              | 0x0        | GPIO4 |
| GPIO_3           | 3      | rw              | 0x0        | GPIO3 |
| GPIO_2           | 2      | rw              | 0x0        | GPIO2 |
| GPIO_1           | 1      | rw              | 0x0        | GPIO1 |
| GPIO_0           | 0      | rw              | 0x0        | GPIO0 |

Back to [Register map](#register-map-summary).

## GPIO_CONFIG

GPIO_CONFIG

Address offset: 0x00000004

Reset value: 0x0000ff00

![gpio_config](md_img/gpio_config.svg)

| Name             | Bits   | Mode            | Reset      | Description |
| :---             | :---   | :---            | :---       | :---        |
| -                | 31:16  | -               | 0x0000     | Reserved |
| GPIO_15_CONFIG   | 15     | rw              | 0x1        | GPIO15_CONFIG |
| GPIO_14_CONFIG   | 14     | rw              | 0x1        | GPIO14_CONFIG |
| GPIO_13_CONFIG   | 13     | rw              | 0x1        | GPIO13_CONFIG |
| GPIO_12_CONFIG   | 12     | rw              | 0x1        | GPIO12_CONFIG |
| GPIO_11_CONFIG   | 11     | rw              | 0x1        | GPIO11_CONFIG |
| GPIO_10_CONFIG   | 10     | rw              | 0x1        | GPIO10_CONFIG |
| GPIO_9_CONFIG    | 9      | rw              | 0x1        | GPIO9_CONFIG |
| GPIO_8_CONFIG    | 8      | rw              | 0x1        | GPIO8_CONFIG |
| GPIO_7_CONFIG    | 7      | rw              | 0x0        | GPIO7_CONFIG |
| GPIO_6_CONFIG    | 6      | rw              | 0x0        | GPIO6_CONFIG |
| GPIO_5_CONFIG    | 5      | rw              | 0x0        | GPIO5_CONFIG |
| GPIO_4_CONFIG    | 4      | rw              | 0x0        | GPIO4_CONFIG |
| GPIO_3_CONFIG    | 3      | rw              | 0x0        | GPIO3_CONFIG |
| GPIO_2_CONFIG    | 2      | rw              | 0x0        | GPIO2_CONFIG |
| GPIO_1_CONFIG    | 1      | rw              | 0x0        | GPIO1_CONFIG |
| GPIO_0_CONFIG    | 0      | rw              | 0x0        | GPIO0_CONFIG |

Back to [Register map](#register-map-summary).
