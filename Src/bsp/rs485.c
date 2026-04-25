#include "tim.h"
#include "usart.h" 
#include "rs485.h"
#include "motor.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

uint16_t error_log[ERROR_LOG_SIZE] = {0};  // 5 ????????? ??????
uint8_t  error_log_index = 0;
uint8_t sendedNotifaction = 0;
uint8_t sendedNotifactionKey = 0;  
uint16_t segments_loaded[3] = {0, 0, 0};
uint8_t rs_segment_count[3] = {0,0,0}; 


void AddLog(uint16_t error_code)
{
    error_log[error_log_index] = error_code;
    error_log_index = (error_log_index + 1) % ERROR_LOG_SIZE;
}
void PrintLastErrors(void)
{
    UART_SendString("=== Error log dump ===\r\n");
    for (int i = 0; i < ERROR_LOG_SIZE; ++i) {
        char line[32];
        sprintf(line, "#%02d -> %04u\r\n", i, error_log[i]);
        UART_SendString(line);
    }
}
// ??????????? CRC-16 Modbus (??????? 0xA001, ????????? ???????? 0xFFFF).
static uint16_t ModbusRTU_CRC(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

// ??????? ????????? ?????????? RS485 ????? UART
static HAL_StatusTypeDef Modbus_ReceiveBytes(UART_HandleTypeDef *huart,
                                            uint8_t *pData,
                                            uint16_t size,
                                            uint32_t timeout)
{
		__HAL_UART_CLEAR_OREFLAG(huart); // Overrun
    __HAL_UART_CLEAR_FEFLAG(huart);  // Framing
    __HAL_UART_CLEAR_NEFLAG(huart);  // Noise
    __HAL_UART_CLEAR_PEFLAG(huart);  // Parity

    // ?????? RDR ?????????? ????????? "????????" ??????
    volatile uint32_t tmp = huart->Instance->DR;
    (void)tmp;
 

    uint16_t bytesReceived = 0;
    while (bytesReceived < size)
    {
        // ?????? ?? 1 ?????, ????? ????? ???? ???????? ?? ??????
        if (HAL_UART_Receive(huart, &pData[bytesReceived], 1, timeout) == HAL_OK)
        {
            bytesReceived++;
        }
        else
        {
            return HAL_ERROR; // ?????? ??? ???????
        }
    }
    return HAL_OK;
}



void UART_SendString(const char *msg)
{ 
	if (DEBUG) HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000); 
}
// -------------------------------------------------------------------------
// ?????????? ????????? Modbus (?????? ??? Write Multiple Registers).
// -------------------------------------------------------------------------
void RS485_USARTx_ProcessModbus(void)
{
    // ????? ??? ???????? ??????
		UART_SendString("MODBUS > start receive\r\n");

    uint8_t rxBuffer[256];
    memset(rxBuffer, 0, sizeof(rxBuffer));

    // 1) ????????? ??????? 6 ???? (Addr, Func, StartHi, StartLo, RegCountHi, RegCountLo).
    if (Modbus_ReceiveBytes(&huart3, rxBuffer, 6, HAL_MAX_DELAY) != HAL_OK)
    {
        UART_SendString("Receive error (header)\r\n");
        return;
    }
		#define MAX_REGISTERS  37

    uint8_t  addr      = rxBuffer[0];
    uint8_t  func      = rxBuffer[1];
    uint16_t startAddr = (uint16_t)(rxBuffer[2] << 8) | rxBuffer[3];
    uint16_t regCount  = (uint16_t)(rxBuffer[4] << 8) | rxBuffer[5];
		
    // ???? ????? ?? ????????? — ?????????? (???????).
    if (addr != SLAVE_ADDRESS)
    {
        return;
    }

    // -------------------------------------------------------------------------
    // ????????? ??????? 0x03 (Read Holding Registers)
    if (func == 0x03)
    { 
				
        // ?????????? 2 ????? CRC (????? 8 ????).
        if (Modbus_ReceiveBytes(&huart3, &rxBuffer[6], 2, HAL_MAX_DELAY) != HAL_OK)
        {
            UART_SendString("Receive error (CRC)\r\n");
						AddLog(0x4000);
            return;
        }

        // ?????????? 16-??? CRC ?? ????????? ???? ????
        uint16_t crc_recv = (uint16_t)(rxBuffer[7] << 8) | rxBuffer[6];

        // ??????? CRC ?? ?????? 6 ?????? ???????
        uint16_t crc_calc = ModbusRTU_CRC(rxBuffer, 6);

        if (crc_recv != crc_calc)
        {
            UART_SendString("CRC error (read)\r\n");
						AddLog(0x4001);
            return;
        }

        // ????????, ??? ????? ???????? ? 0 ?? 3 (4 ??).
        if (startAddr + regCount > MAX_REGISTERS) {
						UART_SendString("Register out of range\r\n");
						AddLog(0x4002);
						return;
				}

        uint8_t txBuf[5 + MAX_REGISTERS * 2]; // Addr, Func, ByteCount, Data[], CRC(2)
				txBuf[0] = SLAVE_ADDRESS;
				txBuf[1] = 0x03;
				txBuf[2] = regCount * 2; // ByteCount
				RefreshReadyFlag();
				for (uint16_t i = 0; i < regCount; i++) {
						uint16_t regVal = 0;

						switch (startAddr + i) {
								case 0:  regVal = 0; break;
								case 1:  regVal = 0; break;
								case 2:  regVal = 0; break;
								case 3:  regVal = 0; break;
								case 4:  regVal = ena; break;
								case 5:  regVal = alarm; break;
								case 6:  regVal = ready; break;
								case 7:  regVal = is_on; break;
								case 8:  regVal = MotionStatusFlags(); break;
								case 9:  regVal = rs_mode; break;  
								case 11: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 1) % ERROR_LOG_SIZE]; break;
								case 12: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 2) % ERROR_LOG_SIZE]; break;
								case 13: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 3) % ERROR_LOG_SIZE]; break;
								case 14: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 4) % ERROR_LOG_SIZE]; break;
								case 15: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 5) % ERROR_LOG_SIZE]; break; 
								case 16: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 6) % ERROR_LOG_SIZE]; break; 
								case 17: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 7) % ERROR_LOG_SIZE]; break; 
								case 18: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 8) % ERROR_LOG_SIZE]; break; 
								case 19: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 9) % ERROR_LOG_SIZE]; break; 
								case 20: regVal = error_log[(error_log_index + ERROR_LOG_SIZE - 10) % ERROR_LOG_SIZE]; break; 
	/*							case 21: regVal = (uint16_t)HAL_GPIO_ReadPin(GPIOH, LimitSwitch1_Pin); break;
								case 22: regVal = (uint16_t)HAL_GPIO_ReadPin(GPIOH, LimitSwitch2_Pin); break;
								case 23: regVal = (uint16_t)HAL_GPIO_ReadPin(GPIOH, LimitSwitch3_Pin); break;
								case 24: regVal = (uint16_t)HAL_GPIO_ReadPin(GPIOA, LimitSwitch4_Pin); break;
*/	
								case 21: regVal = (uint16_t)limitSwitches[0]; break;
								case 22: regVal = (uint16_t)limitSwitches[1]; break;
								case 23: regVal = (uint16_t)limitSwitches[2]; break;
								case 24: regVal = (uint16_t)limitSwitches[3]; break;
                                case 25: regVal = (uint16_t)((uint32_t)axis_total_counter[0] & 0xFFFFU); break;
                                case 26: regVal = (uint16_t)(((uint32_t)axis_total_counter[0] >> 16) & 0xFFFFU); break;
                                case 27: regVal = (uint16_t)((uint32_t)axis_total_counter[1] & 0xFFFFU); break;
                                case 28: regVal = (uint16_t)(((uint32_t)axis_total_counter[1] >> 16) & 0xFFFFU); break;
                                case 29: regVal = (uint16_t)((uint32_t)axis_total_counter[2] & 0xFFFFU); break;
                                case 30: regVal = (uint16_t)(((uint32_t)axis_total_counter[2] >> 16) & 0xFFFFU); break;
                                case 33: regVal = 1; break; // protocol_version.major
                                case 34: regVal = 0; break; // protocol_version.minor
                                case 35: regVal = 1; break; // firmware_version.major
                                case 36: regVal = 0; break; // firmware_version.minor

								

								default: regVal = 0xFFFF; break; // Unknown register
						} 
						txBuf[3 + i * 2]     = (regVal >> 8) & 0xFF;
						txBuf[3 + i * 2 + 1] = regVal & 0xFF;
				} 

        // ??????? CRC ?? ?????? 11 ??????
        uint16_t crc = ModbusRTU_CRC(txBuf, 3 + regCount * 2);
				txBuf[3 + regCount * 2] = crc & 0xFF;
				txBuf[4 + regCount * 2] = (crc >> 8) & 0xFF; 

        // ??????????? ????????? ? ???????? ? ??????????
        TX_MODE();
				HAL_UART_Transmit(&huart3, txBuf, 5 + regCount * 2, 1000);
				RX_MODE(); 
        return;
    } else  if (func == 0x10)
    {
        // 3) ?????? ????????? ????????? ???? (ByteCount):
        //    ByteCount = regCount * 2 (?.?. ?????? ??????? ?? 2 ?????).
        if (Modbus_ReceiveBytes(&huart3, &rxBuffer[6], 1, HAL_MAX_DELAY) != HAL_OK)
        {
            UART_SendString("Receive error (bytecount)\r\n");
						AddLog(0x4003);
            return;
        }
        uint8_t byteCount = rxBuffer[6];

        // ????????, ?????? ?? ??? ? rxBuffer
        // 6 ???? ??? ??????? + 1 ???? byteCount => ??? 7
        // ???? ??? ???????? byteCount + 2 (CRC)
        uint16_t toRead = (uint16_t)(byteCount + 2);
        if (7 + toRead > sizeof(rxBuffer))
        {
            UART_SendString("Packet too large for buffer\r\n");
						AddLog(0x4004);
            return;
        }

        // ?????????? ?????? ????????? (byteCount) + 2 ????? CRC
        if (Modbus_ReceiveBytes(&huart3, &rxBuffer[7], toRead, HAL_MAX_DELAY) != HAL_OK)
        {
            UART_SendString("Receive error (data+CRC)\r\n");
						AddLog(0x4005);
            return;
        }

        // ????? ????? ?????:
        uint16_t frameLength = 7 + toRead; // 7 ??? ???????? + (byteCount+2)

        // ????????? 2 ????? — ??? CRC
        uint16_t crc_recv = (uint16_t)(rxBuffer[frameLength - 1] << 8)
                           | (rxBuffer[frameLength - 2]);

        // ??????? CRC ?? ?????, ????? ???? ???? CRC
        uint16_t crc_calc = ModbusRTU_CRC(rxBuffer, frameLength - 2);
        if (crc_calc != crc_recv)
        {
            UART_SendString("CRC Error (write)\r\n");
						AddLog(0x4006);
            return;
        }

        // ????????, ??? byteCount == regCount * 2
        if (byteCount != regCount * 2)
        {
            UART_SendString("ByteCount mismatch\r\n");
						AddLog(0x4007);
            return;
        }  

			if (startAddr >= 400 && (startAddr - 400) % SEGMENT_REGISTERS == 0 && regCount % SEGMENT_REGISTERS == 0)
			{
					uint16_t segment_offset = (startAddr - 400) / SEGMENT_REGISTERS;
					uint16_t segment_count  = regCount / SEGMENT_REGISTERS;

					if ((segment_offset + segment_count) > SEGMENTS_MAX) {
							UART_SendString("Too many total segments\r\n");
							return;
					}

					// ???????? ????? ? ???????? ? ?????? offset
					for (uint16_t i = 0; i < segment_count; ++i)
					{
							for (uint8_t m = 0; m < 3; ++m)
							{
									uint16_t raw = (rxBuffer[7 + (i * 3 + m) * 2] << 8) | rxBuffer[7 + (i * 3 + m) * 2 + 1];
									angle_segments[segment_offset + i][m] = (int16_t)raw;
							}
					}

					// ????????? segments_loaded ? rs_segment_count ??? ???? ???????
					for (uint8_t m = 0; m < 3; ++m)
					{
							uint16_t new_count = segment_offset + segment_count;
							if (new_count > segments_loaded[m])     segments_loaded[m]     = new_count;
							if (new_count > rs_segment_count[m])    rs_segment_count[m]    = new_count;
					}

					UART_SendString("Angle segment batch written OK\r\n");
			} 
			else if (startAddr == 400)
			{
					UART_SendString("Invalid segment count or overflow.\r\n");
			} 
			else if (startAddr == 300 && regCount == 2)
			{
					uint16_t seg_count = (rxBuffer[7] << 8) | rxBuffer[8];
					uint16_t time_ms   = (rxBuffer[9] << 8) | rxBuffer[10];

					// ???????? ??????
					if (seg_count > 200) {
							UART_SendString("Too many segments, limit 200\r\n");
							return;
					}

					// ????????: ???? ?? ??? ????????? ?? ?????
					if (segments_loaded[0] < seg_count || segments_loaded[1] < seg_count || segments_loaded[2] < seg_count) {
							UART_SendString("Segments not fully received yet\r\n");
							return;
					}

					BuildProfileFromAngleSegments(angle_segments, seg_count, time_ms);
					UART_SendString("Trajectory started via command\r\n");
					
					AddLog(0x1009);
			}  
			else if (startAddr == 301 && regCount == 1)
			{
					uint16_t goCmd = (rxBuffer[7] << 8) | rxBuffer[8];
					if (goCmd == 1) {
							GoToHome();
							UART_SendString("Go to home command accepted\r\n");
							AddLog(0x1010);
					}
			}
			else if (startAddr == 302 && regCount == 2)
			{
					uint16_t goCmd = (rxBuffer[7] << 8) | rxBuffer[8];
					uint16_t time_ms   = (rxBuffer[9] << 8) | rxBuffer[10];
					if (goCmd == 1) {
							Motor4_Run180Cycle(time_ms);
							UART_SendString("Running 4 channel\r\n");
							AddLog(0x1010);
					}
			}
			else if (startAddr == 303 && regCount == 1)
			{
					if (((rxBuffer[7] << 8) | rxBuffer[8]) == 1)
					{
							if (EmergencyStopRequest())
							{
									AddLog(0x1011);
							}
							else
							{
									AddLog(0x2011);
							}
					}
			}
			else if (startAddr == 406 && regCount == 1)
			{
					if (((rxBuffer[7]<<8)|rxBuffer[8]) == 1)
                    {
                            if (CalibrationRequest()) {
                                    AddLog(0x1012);
                            } else {
                                    AddLog(0x2012);
                            }
                    }
			}




			 
			else if (startAddr == 407 && regCount == 1)
			{
					if (((rxBuffer[7] << 8) | rxBuffer[8]) == 1)
					{
							if (!MoveToWorkTopCenterRequest())
							{
									AddLog(0x2017);
							}
					}
			}
        uint8_t txReply[8];
        txReply[0] = SLAVE_ADDRESS;
        txReply[1] = 0x10;
        txReply[2] = rxBuffer[2]; // StartHi
        txReply[3] = rxBuffer[3]; // StartLo
        txReply[4] = rxBuffer[4]; // RegCountHi
        txReply[5] = rxBuffer[5]; // RegCountLo

        // CRC
        uint16_t crc = ModbusRTU_CRC(txReply, 6);
        txReply[6] = (uint8_t)(crc & 0xFF);      // CRC Lo
        txReply[7] = (uint8_t)((crc >> 8) & 0xFF); // CRC Hi

        // ??????????
        TX_MODE();
        HAL_UART_Transmit(&huart3, txReply, 8, 1000);
        RX_MODE(); 
        return;
    }

    // ???? ?????? ?????? ??????? — ???? ?? ????????????.
    // ????? ???? ?? ????????? Exception.
    UART_SendString("Unsupported function\r\n");
		AddLog(0x4008);
}
