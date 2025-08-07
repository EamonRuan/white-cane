#include "ti_msp_dl_config.h"
#include "humidity.h"
#include <stdio.h>
#include "oled_hardware_i2c.h"
extern uint8_t oled_buffer[32];
volatile bool gCheckADC;
volatile uint16_t gAdcResult;
uint16_t siducnt=0;
void humidity_GetStatus()
{
     DL_ADC12_startConversion(ADC12_0_INST);

     gAdcResult = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);

        //  printf("湿度 =%d\r\n", gAdcResult);

         sprintf((char *)oled_buffer, "sd:%d  ",gAdcResult );
        OLED_ShowString(0,0,oled_buffer,16);
        if (gAdcResult > 2500) {
            siducnt=0;
             DL_GPIO_clearPins(GPIO_Ultrasonic_PIN_B1_PORT, GPIO_Ultrasonic_PIN_B1_PIN);
        } else {
            siducnt++;
            if(siducnt>2)
            {
                siducnt=0;
                  DL_GPIO_setPins(GPIO_Ultrasonic_PIN_B1_PORT, GPIO_Ultrasonic_PIN_B1_PIN);
            }
         
        }
      
        DL_ADC12_enableConversions(ADC12_0_INST);

}
void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            gCheckADC = true;
            break;
        default:
            break;
    }
}
