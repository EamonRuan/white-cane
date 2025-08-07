#include "ti_msp_dl_config.h"
#include <stdint.h>




extern void init_crc_table(void);
static unsigned int crc32(const unsigned char *buf, unsigned int size);
unsigned short int calculate_average(unsigned short int *data, int length);
int detect_step(unsigned short int *distances, int length);
extern void dtof_Task(void);
void classify_obstacle(unsigned short int *distances, int length);
extern unsigned short int distances[64];
extern unsigned short int *data;
extern unsigned int crc_data;
extern unsigned char uartReadBuff[300];
extern bool step_flag;
extern unsigned int len;
extern uint16_t juli;
extern unsigned short int distances[64];
extern void Motor_Enable();
extern void Set_Speed_Mode_CW(void);
extern uint8_t r_flag;
extern uint8_t l_flag;
extern uint16_t rlstime;
extern uint16_t llstime;
extern uint8_t zx_flag;
extern uint8_t uart_flag_yuyin;

