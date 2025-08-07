#include "ti_msp_dl_config.h"
#include "motor.h"
void Motor_Enable(void) //使能电机
{
    uint8_t buffer[] = {0x01,0xF3,0xAB,0x01,0x00,0x6B};
    size_t len = sizeof(buffer);

    for (size_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART0, buffer[i]);  // UART0 是实际 UART 寄存器名
    }
}
void Set_Speed_Mode_CW(void) //电机以200转顺时针旋转
{
    uint8_t buffer[] = {0x01,0xF6,0x00,0x00,0xC8,0x00,0x00,0x6B};
    size_t len = sizeof(buffer);

    for (size_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART0, buffer[i]);  // UART0 是实际 UART 寄存器名
    }
}
void Set_Speed_Mode_CWW(void) //电机以200转逆时针旋转
{
    uint8_t buffer[] = {0x01,0xF6,0x01,0x00,0xC8,0x00,0x00,0x6B};
    size_t len = sizeof(buffer);

    for (size_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART0, buffer[i]);  // UART0 是实际 UART 寄存器名
    }
}
void stop_motor(void)//停止旋转
{
uint8_t buffer[] = {0x01, 0xFE, 0x98, 0x00, 0x6B};
    size_t len = sizeof(buffer);

    for (size_t i = 0; i < len; i++) {
        DL_UART_transmitDataBlocking(UART0, buffer[i]);  // UART0 是实际 UART 寄存器名
    }
}
