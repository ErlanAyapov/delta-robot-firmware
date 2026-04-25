#ifndef __RS485_H
#define __RS485_H  
#include <stdint.h>
#define SEGMENT_REGISTERS 3  // ?? 3 ???????? (X,Y,Z) ?? ???????

extern uint8_t sendedNotifaction;
extern uint8_t sendedNotifactionKey;
extern uint8_t rs_segment_count[3]; 
extern uint16_t segments_loaded[3];
void PrintLastErrors(void);
void AddLog(uint16_t error_code);
void UART_SendString(const char *msg);
void RS485_USARTx_ProcessModbus(void);
#endif /* __RS485_H */
