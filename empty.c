#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "string.h"
#include "board.h"
#include <stdio.h>
#include "bsp_mpu6050.h"
#include "humidity.h"
#include "dtof.h"
#include "oled_hardware_i2c.h"
#include "interrupt.h"
// #include "motor.h"
uint32_t dist;
volatile bool gCheckADC;
volatile uint16_t gAdcResult;
// uint8_t gEchoData = 1;
// uint8_t ch = 1;
uint8_t oled_buffer[32];
uint16_t shidutime=0;
uint8_t  shidu_flag=0;
uint8_t gps_flag=0;
uint16_t time5s=0;

#define GPS_Buffer_Length   256
#define USART_REC_LEN  		200  	//定义最大接收字节数 200
#define EN_USART1_RX 		1		//使能（1）/禁止（0）串口1接收
#define false 0
#define true 1
//定义数组长度
#define UTCTime_Length 11
#define latitude_Length 11
#define N_S_Length 2
#define longitude_Length 12
#define E_W_Length 2 	  	

uint8_t  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
uint16_t USART_RX_STA;         		//接收状态标记	
uint8_t  point1 = 0;
uint8_t  Res;
uint8_t gEchoData;


uint8_t Get_dma_buffer[137];

typedef struct SaveData 
{
	char GPS_Buffer[GPS_Buffer_Length];
	char isGetData;		//是否获取到GPS数据
	char isParseData;	//是否解析完成
	char UTCTime[UTCTime_Length];		//UTC时间
	char latitude[latitude_Length];		//纬度
	char N_S[N_S_Length];		//N/S
	char longitude[longitude_Length];		//经度
	char E_W[E_W_Length];		//E/W
	char isUsefull;		//定位信息是否有效
} _SaveData;
_SaveData Save_Data;

//函数声明区域
void CLR_Buf(void);
void clrStruct();
void parseGpsBuffer();
void printGpsBuffer();
void errorLog(int num);
// void sendGpsToESP32(const char *lat, const char *lng);


int main(void)
{
    SYSCFG_DL_init();
    NVIC_ClearPendingIRQ(UART_DTOF_INST_INT_IRQN);
    // NVIC_EnableIRQ(UART_DTOF_INST_INT_IRQN);
    // OLED_Init();
    board_init();
	//MPU6050_Init();
	

	NVIC_ClearPendingIRQ(UART_GPS_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_GPS_INST_INT_IRQN);

    CLR_Buf();
	clrStruct();//清空
	
	
    NVIC_EnableIRQ(TIMER_MPU6050_INST_INT_IRQN);//超声波探测超时中断使能
    DL_TimerG_startCounter(TIMER_MPU6050_INST);
    NVIC_EnableIRQ(TIMER_Ultrasonic_INST_INT_IRQN);//超声波探测超时中断使能
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);//湿度传感器adc中断使能

    init_crc_table();//激光雷达
	Motor_Enable();
	DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)(&UART_DTOF_INST->RXDATA));
	DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&uartReadBuff);
	DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID,137);
	DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
	NVIC_EnableIRQ(UART_DTOF_INST_INT_IRQN);
	DL_GPIO_clearPins(GPIO_Ultrasonic_PIN_B1_PORT, GPIO_Ultrasonic_PIN_B1_PIN);
	DL_GPIO_clearPins(GPIO_Ultrasonic_PIN_B0_PORT, GPIO_Ultrasonic_PIN_B0_PIN);
	DL_GPIO_clearPins(GPIO_Ultrasonic_PIN_B1_PORT, GPIO_Ultrasonic_PIN_B2_PIN);
	     

	 /* Don't remove this! */
	OLED_Init();
    Interrupt_Init();

    while (1) 
    {
        dtof_Task();
		if(gps_flag)
		{
			parseGpsBuffer();//GPS处理功能
			printGpsBuffer();//打印串口信息
			gps_flag=0;
			time5s=0;
		}
		
      	if(shidu_flag==1)
    	{
          shidu_flag=0;
          shidutime=0;
      	  humidity_GetStatus();
        }
    }
}
// void UART_DTOF_INST_IRQHandler(void)
// {

//     switch (DL_UART_Main_getPendingInterrupt(UART_DTOF_INST)) {
//         case DL_UART_MAIN_IIDX_RX:
        
//              uartReadBuff[len++]=DL_UART_Main_receiveData(UART_DTOF_INST);
    
//         if (len>137) {
//             len=0;
        
//         }
//             break;
//         default:
//             break;
//     }
// }


void UART_DTOF_INST_IRQHandler(void)
{
    switch(DL_UART_Main_getPendingInterrupt(UART_DTOF_INST))
	{
		case DL_UART_MAIN_IIDX_DMA_DONE_RX:
			uart_flag_yuyin = 1;
		break;
		default:
			break;
	}
}
void UART_GPS_INST_IRQHandler(void)
{

    //gEchoData = DL_UART_Main_receiveData(UART_0_INST);//接收数据
    //DL_UART_Main_transmitData(UART_0_INST, gEchoData);//发送数据
    // 判断是否有 RX 数据就绪（相当于 STM32 的 USART_IT_RXNE）

    if (DL_UART_Main_getRawInterruptStatus(UART3,DL_UART_MAIN_INTERRUPT_RX))
    {
        Res = DL_UART_Main_receiveData(UART3); //读取数据
        //DL_UART_Main_clearInterruptStatus(UART1, DL_UART_MAIN_INTERRUPT_RX); //清除 RX 中断标志

        if (Res == '$') //如果遇到帧头 '$' 就重置索引
        {
            point1 = 0;
        }

        if (point1 < USART_REC_LEN)//小于最大长度
        {
            USART_RX_BUF[point1++] = Res;//存入数组
        }
        else // 防止溢出
        {
            point1 = USART_REC_LEN - 1;
        }

        if (USART_RX_BUF[0] == '$' 
            && USART_RX_BUF[4] == 'M' 
            && USART_RX_BUF[5] == 'C'
            && Res == '\n')
        {
            // 拷贝整帧到 Save_Data.GPS_Buffer
            memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);
            memcpy(Save_Data.GPS_Buffer, USART_RX_BUF, point1);
            Save_Data.isGetData = true;
            // 重置，准备下一帧
            point1 = 0;
            memset(USART_RX_BUF, 0, USART_REC_LEN);
        }
		
		if(point1 >= USART_REC_LEN)
		{
			point1 = USART_REC_LEN;//防止超过最大长度
		}
    }
}       
void CLR_Buf(void)// 串口缓存清理
{
	memset(USART_RX_BUF, 0, USART_REC_LEN);      //清空
	point1 = 0;                    
}

//以下是和GPS相关的参数配置  清空所有信息
void clrStruct()
{
	Save_Data.isGetData = false;
	Save_Data.isParseData = false;
	Save_Data.isUsefull = false;
	memset(Save_Data.GPS_Buffer, 0, GPS_Buffer_Length);      //清空
	memset(Save_Data.UTCTime, 0, UTCTime_Length);
	memset(Save_Data.latitude, 0, latitude_Length);
	memset(Save_Data.N_S, 0, N_S_Length);
	memset(Save_Data.longitude, 0, longitude_Length);
	memset(Save_Data.E_W, 0, E_W_Length);
	
}
void errorLog(int num)
{
	//printf("ERROR%d\r\n", num);
    //Save_Data.isGetData = false;
}

//发送数据到esp32
void sendGpsToESP32(const char *lat, const char *lng)
{
    char sendBuf[100];
    sprintf(sendBuf, "<START>LAT:%s,LNG:%s<END>\r\n", lat, lng);  // 注意格式！

    for (int i = 0; sendBuf[i] != '\0'; i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_GPS_INST, sendBuf[i]);
    }
}


//--串口解析函数
void parseGpsBuffer()
{
		//printf("**************\r\n");
	char *subString;
	char *subStringNext;
	char i = 0;
	if (Save_Data.isGetData)
	{
		Save_Data.isGetData = false;
		//printf("**************\r\n");
		//printf("%s",Save_Data.GPS_Buffer);//打印信息

		for (i = 0 ; i <= 6 ; i++)
		{
			if (i == 0)
			{
				if ((subString = strstr(Save_Data.GPS_Buffer, ",")) == NULL)
					errorLog(1);	//解析错误  打印报错信息
			}
			else
			{
				subString++;
				if ((subStringNext = strstr(subString, ",")) != NULL)
				{
					char usefullBuffer[2]; 
					switch(i)
					{
						case 1:memcpy(Save_Data.UTCTime, subString, subStringNext - subString);break;	//获取UTC时间
						case 2:memcpy(usefullBuffer, subString, subStringNext - subString);break;	//获取UTC时间
						case 3:memcpy(Save_Data.latitude, subString, subStringNext - subString);break;	//获取纬度信息
						case 4:memcpy(Save_Data.N_S, subString, subStringNext - subString);break;	//获取N/S
						case 5:memcpy(Save_Data.longitude, subString, subStringNext - subString);break;	//获取经度信息
						case 6:memcpy(Save_Data.E_W, subString, subStringNext - subString);break;	//获取E/W

						default:break;
					}

					subString = subStringNext;
					Save_Data.isParseData = true;
					if(usefullBuffer[0] == 'A')
						Save_Data.isUsefull = true;
					else if(usefullBuffer[0] == 'V')
						Save_Data.isUsefull = false;
				}
				else
				{
					errorLog(2);	//解析错误
				}
			}
		}
	}
}
//串口打印函数
void printGpsBuffer()
{
	
	if (Save_Data.isParseData)//代表有数据 非空  可以直接打印
	{
		Save_Data.isParseData = false;
		
		printf("Save_Data.UTCTime = ");
		printf("%s",Save_Data.UTCTime);
		printf("\r\n");
		
		if(Save_Data.isUsefull)
		{
			//sendGpsToESP32("3448.66504", "11330.03177");//发送测试
			Save_Data.isUsefull = false;
			//printf("Save_Data.latitude = ");
			//printf("%s",Save_Data.latitude);
			//printf("\r\n");

			//printf("Save_Data.N_S = ");
			//printf("%s",Save_Data.N_S);
			//printf("\r\n");

			//printf("Save_Data.longitude = ");
			//printf("%s",Save_Data.longitude);
			//printf("\r\n");

			//printf("Save_Data.E_W = ");
			//printf("%s",Save_Data.E_W);
			//printf("\r\n");

			sendGpsToESP32(Save_Data.latitude, Save_Data.longitude);
			//发送数据给esp32模块
		}
		else//否则打印不能使用
		{
			//printf("GPS DATA is not usefull!\r\n");
		}
	}
}


void TIMER_MPU6050_INST_IRQHandler(void)
{
    
	time5s++;
    shidutime++;
    if(shidutime>=20)
    {
        shidutime=0;
        shidu_flag=1;
    }
  	if(time5s>=100)
    {
		gps_flag=1;
		time5s=0;
 	}
    if(r_flag==1)
    {	
		zx_flag=1;
		rlstime++;
		if(rlstime>=40)
		{
			rlstime=0;
			r_flag=0;
			zx_flag=0;
		}
   }
    if(l_flag==1)
    {
		zx_flag=1;
		llstime++;
		if(llstime>=40)
		{
			llstime=0;
			l_flag=0;
			zx_flag=0;
		}
   }
}