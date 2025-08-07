#include "ultrasonic.h"
#include "stdio.h"
#include "board.h"
uint8_t overflowFlag;

uint32_t getUltrasonicDist(void)
{
    uint32_t cnt, dist;
    DL_GPIO_setPins(GPIO_Ultrasonic_PIN_Trig_PORT, GPIO_Ultrasonic_PIN_Trig_PIN);
    delay_cycles(320000);
    DL_GPIO_clearPins(GPIO_Ultrasonic_PIN_Trig_PORT, GPIO_Ultrasonic_PIN_Trig_PIN);

    while(!DL_GPIO_readPins(GPIO_Ultrasonic_PIN_Echo_PORT, GPIO_Ultrasonic_PIN_Echo_PIN));

    overflowFlag = 0;
    DL_TimerG_setTimerCount(TIMER_Ultrasonic_INST, 0);
    DL_TimerG_startCounter(TIMER_Ultrasonic_INST);

    while(DL_GPIO_readPins(GPIO_Ultrasonic_PIN_Echo_PORT, GPIO_Ultrasonic_PIN_Echo_PIN) && !overflowFlag);

    DL_TimerG_stopCounter(TIMER_Ultrasonic_INST);

    if(overflowFlag)
    { 

        dist = 400;
    }
    else 
    {
       cnt = DL_TimerG_getTimerCount(TIMER_Ultrasonic_INST);
        dist = cnt*0.017;
      
    }
    return dist;
}

void ultrasonic_GetStatus()
{
    int dist =  getUltrasonicDist();
             
    printf("Dist: %d cm\r\n",dist);
    
    if( dist<=60)
    {
        
        if(dist<=60)
        {
            // delay_ms(20);
                if(dist<60)
                DL_GPIO_setPins(GPIO_Ultrasonic_PIN_B0_PORT, GPIO_Ultrasonic_PIN_B0_PIN);
            
        } 
    }
    else if(dist!=0&&dist>60)
    {
        //delay_ms(20);
        if(dist!=0&&dist>60)
        DL_GPIO_clearPins(GPIO_Ultrasonic_PIN_B0_PORT, GPIO_Ultrasonic_PIN_B0_PIN);
        
    }  
}


void TIMER_Ultrasonic_INST_IRQHandler(void)
{
   
    switch (DL_TimerG_getPendingInterrupt(TIMER_Ultrasonic_INST)) {
        case DL_TIMER_IIDX_LOAD:
            overflowFlag = 1;
            break;
        default:
            break;
    }
}
