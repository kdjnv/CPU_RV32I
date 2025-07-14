#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <regs_GPIO.h>
#include <regs_I2C.h>
#include <LIB_TPWM.h>
#include <regs_UART.h>
#include <regs_SPI.h>
#include <regs_TIME_BASE.h>

#define MASK_ON_EN 0x100000
#define MASK_ON_STRX 0x200000
#define MASK_MODE_RX 0x400000
#define MASK_ADDNUMBY_RX 0x400


#define MASK_DIV 0x00040000
#define DIVSPI   270


void uart_send_char(uint8_t my_char){
    UART->U_TXDATA = my_char;
    UART->U_CTRL_bf.STRTX = 1;
    while(UART->U_STAT_bf.TBUSY == 1){};
    UART->U_CTRL_bf.STRTX = 0;
}

void uart_send_str(uint8_t *my_str){
    for (uint8_t i = 0; my_str[i] != '\0'; i++){
        uart_send_char(my_str[i]);
    }
}

void uart_sendint(uint32_t lux) {
    char buffer[16] = {};
    int i = 0, temp = 0;
    if (lux == 0) buffer[i++] = '0';
    while(lux > 0){
        buffer[i++] = lux%10 + '0';
        lux = lux /10;
    }
    for(int j = 0 ; j < i/2; j++){
        temp = buffer[j];
        buffer[j] = buffer[i-1-j];
        buffer[i-1-j] = temp;
    }
    uart_send_str(buffer);
}

void SPI_send(uint64_t data, int length){
    SPI->SPI_TX_HIGH = data >> 32;
    SPI->SPI_TX_LOW = data;
    SPI->CONTROL_REG |= 1 << 17;

    while (SPI->STATUS_REG_bf.BUSY){};
    SPI->CONTROL_REG &= ~(1 << 17);
}


void delay_ms(uint64_t time_ms){
    while(time_ms--){
        for (volatile uint64_t i = 0; i <= 27000/49; i++){
           // i = i;
        }
    }
}
void Set_Counter(uint32_t value){
    TMR->TIM_LOAD = value;
    TMR->TIM_CONFIG = 0x00000023;
    while(TMR->TIM_STATUS_bf.LD != 1){}
    TMR->TIM_CONFIG = 0x00000003;
}
void delay_tim_ms(uint64_t time_ms){
    while(time_ms--){
        Set_Counter(0);
        while(TMR->TIM_COUNTER_bf.CNT < 1000){
        }
    }
}
void delay_tim_us(uint64_t time_us){
    Set_Counter(0);
    while(TMR->TIM_COUNTER_bf.CNT < time_us){
    }
}

void INIT_I2C(){
    I2C->I2C_CONTROL &= ~(0x1FF << 0);      // Clear CLKDIV (bits 9:0)
    I2C->I2C_CONTROL |= (270 & 0x3FF) << 0; // Set CLKDIV = 270

    I2C->I2C_CONTROL &= ~(0x7 << 25);       // Clear NUMBTX (bits 25:23)
    I2C->I2C_CONTROL |= (1 & 0x7) << 23;    // Set NUMBTX = 1

    I2C->I2C_CONTROL &= ~(0x7 << 10);       // Clear NUMBRX (bits 12:10)
    I2C->I2C_CONTROL |= (2 & 0x7) << 10;    // Set NUMBRX = 2

    I2C->I2C_CONTROL &= ~(0xF << 13);       // Clear DUTYSCL (bits 16:13)
    I2C->I2C_CONTROL |= (5 & 0xF) << 13;    // Set DUTYSCL = 5

    I2C->I2C_CONTROL &= ~(1 << 17);         // Clear RSE

    I2C->I2C_CONTROL &= ~(0x3 << 18);       // Clear LENADD (bits 19:18)

    I2C->I2C_CONTROL |= 1 << 20;            // Set EN = 1

    I2C->I2C_CONTROL &= ~(1 << 21);         // Clear STRX

    I2C->I2C_CONTROL &= ~(1 << 22);         // Clear MODE

    I2C->I2C_CONTROL &= ~(1 << 26);         // Clear RST

    I2C->I2C_CONTROL &= ~(0x1F << 27);      // Clear SEML (bits 31:27)
    I2C->I2C_CONTROL |= (5 & 0x1F) << 27;   // Set SEML = 5
}
void INIT_TPWM(){
    TPWM_MakeEN(1);
    TPWM_MakePOL(1);
    TPWM_MakeNUM(4);

    TPWM_MakeSEL1(0); 
    TPWM_MakeSEL2(1); 
    TPWM_MakeSEL3(2); 
    TPWM_MakeSEL4(3);
    TPWM_MakeDIV(27);

    TPWM_MakePE(1, 100);
    TPWM_MakePE(2, 100);
    TPWM_MakePE(3, 100);
    TPWM_MakePE(4, 100);

    TPWM_MakeCP(1, 20);
    TPWM_MakeCP(2, 40);
    TPWM_MakeCP(3, 60);
    TPWM_MakeCP(4, 80);
}
void INIT_TBASE(){
    // TMR->TIM_CONFIG_bf.AR = 1;
    // TMR->TIM_CONFIG_bf.DIR = 0;
    // TMR->TIM_CONFIG_bf.EN = 1;
    // TMR->TIM_CONFIG_bf.UD = 0;
    TMR->TIM_PRESCALER = 27;
    TMR->TIM_PERIOD = 65535;
    TMR->TIM_CONFIG = 0x00000003;
}
void INIT_SPI(){
    // Clear và set MODE (bit 28)
    SPI->CONTROL_REG &= ~(1 << 28);
    SPI->CONTROL_REG |= 0 << 28;  // 0 = master

    // Clear và set DIV (bits 27:18)
    SPI->CONTROL_REG &= ~(0x3FF << 18);
    SPI->CONTROL_REG |= 27 << 18;

    // Clear và set STRX (bit 17)
    SPI->CONTROL_REG &= ~(1 << 17);
    SPI->CONTROL_REG |= 0 << 17;

    // Clear và set MSB (bit 16)
    SPI->CONTROL_REG &= ~(1 << 16);
    SPI->CONTROL_REG |= 1 << 16;

    // Clear và set NUMSS (bits 14:11)
    SPI->CONTROL_REG &= ~(0xF << 11);
    SPI->CONTROL_REG |= 0 << 11;

    // Clear và set CSS (bit 9)
    SPI->CONTROL_REG &= ~(1 << 9);
    SPI->CONTROL_REG |= 1 << 9;

    // Clear và set ENSPI (bit 8)
    SPI->CONTROL_REG &= ~(1 << 8);
    SPI->CONTROL_REG |= 1 << 8;

    // Clear và set CPHA (bit 7)
    SPI->CONTROL_REG &= ~(1 << 7);
    SPI->CONTROL_REG |= 0 << 7;

    // Clear và set CPOL (bit 6)
    SPI->CONTROL_REG &= ~(1 << 6);
    SPI->CONTROL_REG |= 0 << 6;

    // Clear và set BPT (bits 5:0)
    SPI->CONTROL_REG &= ~(0x3F << 0);
    SPI->CONTROL_REG |= 8 << 0;
}
void INIT_UART(){
    UART->U_CTRL &= 0xfffffffe;//EN
    UART->U_CTRL |= 1<<0;

    UART->U_CTRL &= 0xffffff0f;
    UART->U_CTRL |= 15<<4;

    UART->U_CTRL &= 0xffff00ff;//CLK
    UART->U_CTRL |= 27<<8;
}


void Make_address(uint8_t slave_add, uint8_t register_add){
    I2C->I2C_SLAVE_ADDR_bf.ADDRESS = slave_add;
    I2C->I2C_REG_ADDR_bf.ADDS = register_add;
}

uint32_t Read_lux(){
    I2C->I2C_TX_DATA_HIGH = 0x00;
    I2C->I2C_TX_DATA_LOW = 0x10;

    I2C->I2C_CONTROL &= ~(1 << 22);
    I2C->I2C_CONTROL |= (1<<21);
    while (I2C->I2C_STATUS_bf.TS == 0){
    }
    I2C->I2C_CONTROL &= ~(1 << 21); 
    delay_ms(120);
    
    I2C->I2C_CONTROL |= (1 << 22);
    I2C->I2C_CONTROL |= (1 << 21);
    while (I2C->I2C_STATUS_bf.RS == 0){
    }
    I2C->I2C_CONTROL &= ~(1 << 21); 
    delay_ms(10);
    
    return I2C->I2C_RX_DATA_LOW;
}

int ck = 0;
void blinkled(int time_delay){
    GPIO->GPIO_IO_bf.GPIO_13 = !(GPIO->GPIO_IO_bf.GPIO_13);
    delay_tim_ms(time_delay);
}

uint32_t lux = 12;
int cnt = 0;
int duty = 0;
int main()
{
    INIT_I2C();
    INIT_SPI();
    INIT_UART();
    INIT_TPWM();
    INIT_TBASE();
    Make_address(0x23, 0x00);
    uart_send_str("begin\n");
    while (1){
        cnt++;
        if (cnt != 20) delay_tim_ms(20);
        else{
            //uart_send_str("----------------------\r\n\0\r\n\0\r\n\0");
            lux = Read_lux();
            uart_sendint(lux); 
            uart_send_str(" lux\r\n\0");
            if (I2C->I2C_STATUS_bf.ERR == 1) uart_send_str("ERROR\n\0");


            SPI_send(0xab, 8);
            uart_send_str("SPI data rx: ");
            uint32_t data_SPI_rx;
            data_SPI_rx = SPI->SPI_RX_LOW + SPI->SPI_RX_HIGH;
            uart_sendint(data_SPI_rx);
            uart_send_str("\r\n\0");
            uart_send_str("UART_RX_res: ");
            uart_send_char(UART->U_RXDATA_bf.DATA);
            uart_send_str("\r\n\0");
            uart_send_str("\r\n\0\r\n\0\r\n\0");
            cnt = 0;
        }

        duty = duty + 1;
        if (duty >= 60){
            duty = 0;
            blinkled(1);
        } 
        TPWM_MakeCP(1, (0+duty>= 60)?0+duty-60:0+duty);
        TPWM_MakeCP(2, (15+duty>= 60)?15+duty-60:15+duty);
        TPWM_MakeCP(3, (30+duty>= 60)?30+duty-60:30+duty);
        TPWM_MakeCP(4, (45+duty>= 60)?45+duty-60:45+duty);
    }
    return 0;
}