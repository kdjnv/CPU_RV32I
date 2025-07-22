#include <regs_PWM.h>


void TPWM_MakePE(int Num, uint16_t value){
    if (Num == 1){
        PCSR->TPWM_PERIOD1 &= 0xffff0000;
        PCSR->TPWM_PERIOD1 |= value;
    }
    else if (Num == 2){
        PCSR->TPWM_PERIOD1 &= 0x0000ffff;
        PCSR->TPWM_PERIOD1 |= value<<16;        
    }
    if (Num == 3){
        PCSR->TPWM_PERIOD2 &= 0xffff0000;
        PCSR->TPWM_PERIOD2 |= value;
    }
    else if (Num == 4){
        PCSR->TPWM_PERIOD2 &= 0x0000ffff;
        PCSR->TPWM_PERIOD2 |= value<<16;        
    }
}

void TPWM_MakeCP(int Num, uint16_t value){
    if (Num == 1){
        PCSR->TPWM_COMPARE1 &= 0xffff0000;
        PCSR->TPWM_COMPARE1 |= value;
    }
    else if (Num == 2){
        PCSR->TPWM_COMPARE1 &= 0x0000ffff;
        PCSR->TPWM_COMPARE1 |= value<<16;        
    }
    if (Num == 3){
        PCSR->TPWM_COMPARE2 &= 0xffff0000;
        PCSR->TPWM_COMPARE2 |= value;
    }
    else if (Num == 4){
        PCSR->TPWM_COMPARE2 &= 0x0000ffff;
        PCSR->TPWM_COMPARE2 |= value<<16;        
    }
}

void TPWM_MakeEN(int en){
    PCSR->TPWM_CONFIG &= 0xfffffffe;
    PCSR->TPWM_CONFIG |= en<<0;
}

void TPWM_MakePOL(int POL){
    PCSR->TPWM_CONFIG &= 0xffffbfff;
    PCSR->TPWM_CONFIG |= POL<<14;
}

void TPWM_MakeSEL1(int loc){
    PCSR->TPWM_CONFIG &= 0xfffffff9;
    PCSR->TPWM_CONFIG |= loc<<1;    
}

void TPWM_MakeSEL2(int loc){
    PCSR->TPWM_CONFIG &= 0xffffffe7;
    PCSR->TPWM_CONFIG |= loc<<3;    
}

void TPWM_MakeSEL3(int loc){
    PCSR->TPWM_CONFIG &= 0xfffffe7f;
    PCSR->TPWM_CONFIG |= loc<<7;    
}

void TPWM_MakeSEL4(int loc){
    PCSR->TPWM_CONFIG &= 0xfffff9ff;
    PCSR->TPWM_CONFIG |= loc<<9;    
}

void TPWM_MakeNUM(int num){
    PCSR->TPWM_CONFIG &= 0xffffc7ff;
    PCSR->TPWM_CONFIG |= num<<11;    
}

void TPWM_MakeDIV(int div){
    PCSR->TPWM_PRESCALER_bf.DIV = 27;
}

