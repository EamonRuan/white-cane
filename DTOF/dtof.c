#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "board.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include "oled_hardware_i2c.h"



#define DTOF_DATA_LEN       137 // dToF数据长度
#define CRC_LEN             133 // CRC校验长度
#define STEP_THRESHOLD      100 // 台阶检测阈值，单位为毫米
#define STEP_WINDOW_SIZE 8   // 用于计算深度差异的窗口大小
#define STEP_DEPTH_DIFF 50   // 用于识别台阶的深度差异阈值
// 定义障碍物的分类高度
#define LOW_OBSTACLE_THRESHOLD 300  // 矮障碍物的高度阈值
#define HIGH_OBSTACLE_THRESHOLD 600 // 高障碍物的高度阈值
extern uint8_t oled_buffer[32];
unsigned char uartReadBuff[137] = {0}; // UART读取缓冲区
unsigned char crcBuff[300] = {0}; // CRC缓冲区
static unsigned int crc32table[256]; // CRC32表
uint8_t uart_flag_yuyin;

void Motor_Enable(void) //使能电机
{
    uint8_t buffer[] = {0x01,0xF3,0xAB,0x01,0x00,0x6B};
    size_t len = sizeof(buffer);

    for (size_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART2, buffer[i]);  // UART0 是实际 UART 寄存器名
    }
}

void Set_Speed_Mode_CW(void) //电机以200转顺时针旋转
{
    uint8_t buffer[] = {0x01,0xF6,0x00,0x00,0xC8,0x00,0x00,0x6B};
    size_t len = sizeof(buffer);

    for (size_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART2, buffer[i]);  // UART0 是实际 UART 寄存器名
    }
}

void Set_Speed_Mode_CWW(void) //电机以200转逆时针旋转
{
    uint8_t buffer[] = {0x01,0xF6,0x01,0x00,0xC8,0x00,0x00,0x6B};
    size_t len = sizeof(buffer);

    for (size_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART2, buffer[i]);  // UART0 是实际 UART 寄存器名
    }
}

void stop_motor(void)//停止旋转
{
    uint8_t buffer[] = {0x01, 0xFE, 0x98, 0x00, 0x6B};
    size_t len = sizeof(buffer);

    for (size_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART2, buffer[i]);  // UART0 是实际 UART 寄存器名
    }
}

uint8_t r_flag=0;
uint8_t l_flag=0;
uint16_t rlstime=0;
uint16_t llstime=0;
uint8_t zx_flag=0;

void pabduan()
{
    if(r_flag==1)
    {
        Set_Speed_Mode_CWW();
        DL_GPIO_setPins(GPIO_LED_PIN_2_PORT, GPIO_LED_PIN_2_PIN); // GPIO高电平
        DL_GPIO_clearPins(GPIO_LED_PIN_3_PORT, GPIO_LED_PIN_3_PIN); // GPIO高电平
    }
   
    else if(l_flag==1)
    {
        Set_Speed_Mode_CW();
        DL_GPIO_setPins(GPIO_LED_PIN_3_PORT, GPIO_LED_PIN_3_PIN); // GPIO高电平
        DL_GPIO_clearPins(GPIO_LED_PIN_2_PORT, GPIO_LED_PIN_2_PIN); // GPIO高电平
    }
    else if(zx_flag==0) 
    {
        r_flag=0;
        l_flag=0;
        rlstime=0;
        llstime=0;
        stop_motor();
        DL_GPIO_clearPins(GPIO_LED_PIN_3_PORT, GPIO_LED_PIN_3_PIN); // GPIO高电平
        DL_GPIO_clearPins(GPIO_LED_PIN_2_PORT, GPIO_LED_PIN_2_PIN); // GPIO高电平
    }
}

void init_crc_table(void)
{
    unsigned int c;
    unsigned int i, j;
    for (i = 0; i < 256; i++) 
    {
        c = (unsigned int)i;
        for (j = 0; j < 8; j++) 
        {
            if (c & 1)
                c = 0xedb88320L ^ (c >> 1);
            else
                c = c >> 1;
        }
        crc32table[i] = c;
    }
}

static unsigned int crc32(const unsigned char *buf, unsigned int size)
{
    unsigned int  i, crc = 0xFFFFFFFF;

    for (i = 0; i < size; i++) 
    {
        crc = crc32table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8); /* 8: 右移8bit */
    }
    return crc ^ 0xFFFFFFFF;
}

unsigned short int calculate_average(unsigned short int *data, int length)
{
    unsigned int sum = 0;
    for (int i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return sum / length;
}


int sum[100];
int cnt=0;
int ret=0;
int detect_step(unsigned short int *distances, int length) 
{
    ret=0;
    for(int i=0;i<8;i++)
    sum[i]=0;
    for(int i=0;i<= length - STEP_WINDOW_SIZE;i=i+8) 
    {
         for(int j=0;j<8;j++)   
         {
              sum[cnt]+=distances[i+j];
         }   
          // printf("%d    \r\n", sum[cnt]);          
        cnt++;
        if(cnt==8)
        cnt=0;
     
    }
    for(int i=0;i<7;i++)
    if(sum[i]-sum[i+1]>400 && sum[i]-sum[i+1]<1000 && sum[i]>16000 && sum[i]<24000)
    {
         
       ret=1;
       break;
    }
    return ret; // 未检测到台阶
}



int count=0;
unsigned short int distances[64] = {0};
unsigned short int *data = NULL;
unsigned int len = 0;
unsigned int crc_data = 0;

int lsum1,rsum1,lsum2,rsum2,lsum3,rsum3;

void zhuanxiang(void)
{

 lsum1=distances[22];
 lsum2=distances[30];
 lsum3=distances[38];
 rsum1=distances[17];
 rsum2=distances[25];
 rsum3=distances[33];
   
    if(lsum1<20)
    lsum1=2499;
        if(rsum1<20)
        rsum1=2499;

    if(lsum2<20)
    lsum2=2499;
        if(rsum2<20)
        rsum2=2499;

    if(lsum3<20)
    lsum3=2499;
        if(rsum3<20)
        rsum3=2499;
   if(lsum1<2500&&lsum1>200&&rsum1>200&&rsum1<2500&&lsum2<2500&&lsum2>200&&rsum2>200&&rsum2<2500&&lsum3<2500&&lsum3>200&&rsum3>200&&rsum3<2500)
   {
        if(lsum1-rsum1>500&&lsum2-rsum2>500&&lsum2-rsum2>500&&zx_flag==0)
        {
            r_flag=1;
            l_flag=0;
         
           
        }
        else if(rsum1-lsum1&&rsum2-lsum2&&rsum3-lsum3>500&&zx_flag==0)
        {
             r_flag=0;
            l_flag=1;
       
        }
       
    
   }
  
         sprintf((char *)oled_buffer, " %-4d--%-4d  ",lsum1,rsum1 );
        OLED_ShowString(0,2,oled_buffer,16);

          sprintf((char *)oled_buffer, " %-4d--%-4d  ",lsum2,rsum2 );
        OLED_ShowString(0,4,oled_buffer,16);  

        sprintf((char *)oled_buffer, " %-4d--%-4d  ",lsum3,rsum3 );
        OLED_ShowString(0,6,oled_buffer,16);
   
    // lsum=0;
    // rsum=0;
}
/* 
 * 接收dToF数据并处理
 * Receive dToF data and process it
 */
uint8_t juli_cnt=0;
uint8_t juli_flag=0;
void dtof_Task(void)
{
   
     
        if (uart_flag_yuyin) 
        {
            uart_flag_yuyin=0;
            if (uartReadBuff[0] == 0xAA && uartReadBuff[1] == 0x55 && uartReadBuff[132] == 0xFF) 
            {
                memcpy(crcBuff, uartReadBuff, CRC_LEN); // 复制数据到CRC缓冲区
                crc_data = crc32(crcBuff, CRC_LEN); // 计算CRC校验

                if (crc_data == *(uint32_t *)(&uartReadBuff[CRC_LEN])) 
                {
                    for (int i = 0; i < 64; i++) 
                    {
                        data = (uint16_t *)(&uartReadBuff[i * 2 + 4]);
                       distances[i] = *data;
                       //printf("%d ",distances[i]);
                    }
                    
                    uint16_t juli = distances[31];
                     
                 
                    if(juli>20)
                    {
                    //printf("距离%dmm\r\n",juli);
                    sprintf((char *)oled_buffer, "jl%-4d ",juli );
                            OLED_ShowString(70,0,oled_buffer,16);
                    if(juli>1200)
                    {  
                        DL_GPIO_clearPins(GPIO_Ultrasonic_PIN_B0_PORT, GPIO_Ultrasonic_PIN_B0_PIN);
                        juli_cnt=0;
                         //  printf("安全\r\n");
                    }            
                    else
                    {
                        juli_cnt++;
                        if(juli_cnt>=2)
                        {
                            DL_GPIO_setPins(GPIO_Ultrasonic_PIN_B0_PORT, GPIO_Ultrasonic_PIN_B0_PIN);
                            juli_cnt=0;
                        }
                   }

                }
                  
                zhuanxiang();
                pabduan();
                int step_result = detect_step(distances, 64);
                if (step_result == 1) 
                {
                    count++;
                    if(count>5)
                    {
                            DL_GPIO_setPins(GPIO_Ultrasonic_PIN_B1_PORT, GPIO_Ultrasonic_PIN_B2_PIN);
                                // printf("台阶!\r\n");
                            DL_GPIO_clearPins(GPIO_LED_PIN_0_PORT, GPIO_LED_PIN_0_PIN); // GPIO高电平
                            count=0;
                    }
                     
                            
                        // delay_ms(500); // 上行台阶，高电平   
                        
                } 
                else if (step_result ==0) 
                {
                    DL_GPIO_clearPins(GPIO_Ultrasonic_PIN_B1_PORT, GPIO_Ultrasonic_PIN_B2_PIN);
                    count=0;
                        
                    DL_GPIO_setPins(GPIO_LED_PIN_0_PORT, GPIO_LED_PIN_0_PIN); // GPIO高电平
                    //  delay_ms(500); // 下行台阶，高电平     
                }


                    // classify_obstacle(distances, 64); 
                }
            }
            //memset(uartReadBuff, 0, sizeof(uartReadBuff));
           len = 0;
        }
       //delay_ms(10);
    
}
