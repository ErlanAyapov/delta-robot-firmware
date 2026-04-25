#ifndef __MOTOR_H
#define __MOTOR_H 
#include "main.h"
#include "tim.h"
#include "usart.h" 
#include "stm32f4xx_hal.h" 
#include <stdio.h>
extern uint8_t ch;  // ????????? ??????????, ?? ?? ?????????? ??
int fgetc(FILE *stream);
int fputc(int c, FILE *stream);


#ifdef __cplusplus
 extern "C" {
#endif 

/* === Global state (??????????) === */ 
extern uint64_t  g_move_time;
extern uint16_t  g_command; 
extern  uint32_t reverseStepCounter[4];
extern  uint8_t  reversingInProgress[4];
  
extern  uint8_t  g_motorRunning[4];  
extern uint8_t   ena;
extern  uint8_t   alarm;
extern uint8_t   rs_mode;
extern uint8_t   is_on;

extern uint8_t limitSwitches[4];
extern uint8_t ready_motors[4];
extern uint8_t   ready;
extern uint8_t DEBUG;

extern  uint32_t reverseStepCounter[4];
extern  uint8_t  reversingInProgress[4];
extern uint8_t   reverseStepFraction;
 
void setMotorDirection(int motorIndex, uint8_t dir);
void StartBackOffForMotor(int i, int16_t steps);
int MotorDisable(void);
int MotorEnable(void);
int MotorStop(void); 
int MotorStart(void);
int MotorStopSingle(uint8_t motor_id);
int rotateRevers(void);
int rotateForward(void);
extern int16_t REDUCTOR_CONF;
extern int16_t CONTROL_MODE;
extern uint32_t motor_tick[4]; 
extern int16_t angle_segments[200][3];
extern volatile int32_t axis_total_counter[3];
 
extern uint32_t real_time_us;       // ????????? ??????
extern uint8_t time_captured;


#define SEGMENTS   240 
#define SEGMENTS_MAX           200
#define MODBUS_MAX_REGISTERS   (SEGMENTS_MAX * 3)
#define MAX_SEGMENT 12800
// ???????????, ??? TIM8_CLOCK = 168 ???
#define TIM8_INPUT_CLK_HZ 84000000
#define TIM8_PRESCALER     83  // ??? 50 ???, ????????
#define TIM8_CLK_HZ (TIM8_INPUT_CLK_HZ / (TIM8_PRESCALER + 1))

#define BASE_RPM     60UL
#define BASE_FREQ    6400UL  
#define STEPMOTOR_MICRO_STEP       32
#define SLAVE_ADDRESS         0x01
#define ERROR_LOG_SIZE 10
#define STEPS_PER_REV   (200 * STEPMOTOR_MICRO_STEP)
/* ------ ??????? ????????? ------ */ 
#define DEG_TO_STEPS(deg)    ((uint32_t)((deg) * (STEPS_PER_REV / 360.0f) + 0.5f))
#define STEPS_IN_SEGMENT = 20;
/* ------ ?????? TIM-8 ------ */
#define TIM8_ARR_MAX         65535u              // Auto-reload register 
#define STEPMOTOR_TIMx                        TIM8
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIMx_IRQn                   TIM8_CC_IRQn
#define STEPMOTOR_TIMx_IRQHandler             TIM8_CC_IRQHandler

#define STEPMOTOR_TIM_CHANNEL_x               TIM_CHANNEL_1
#define STEPMOTOR_TIM_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOI_CLK_ENABLE()     // Êä³ö¿ØÖÆÂö³å¸øµç»úÇý¶¯Æ÷
#define STEPMOTOR_TIM_PUL_PORT                GPIOI                            // ¶ÔÓ¦Çý¶¯Æ÷µÄPUL-£¨Çý¶¯Æ÷Ê¹ÓÃ¹²Ñô½Ó·¨£©
#define STEPMOTOR_TIM_PUL_PIN                 GPIO_PIN_7                       // ¶øPLU+Ö±½Ó½Ó¿ª·¢°åµÄVCC
#define GPIO_AFx_TIMx                         GPIO_AF3_TIM8

#define STEPMOTOR_DIR_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // µç»úÐý×ª·½Ïò¿ØÖÆ£¬Èç¹ûÐü¿Õ²»½ÓÄ¬ÈÏÕý×ª
#define STEPMOTOR_DIR_PORT                    GPIOF                            // ¶ÔÓ¦Çý¶¯Æ÷µÄDIR-£¨Çý¶¯Æ÷Ê¹ÓÃ¹²Ñô½Ó·¨£©
#define STEPMOTOR_DIR_PIN                     GPIO_PIN_2                       // ¶øDIR+Ö±½Ó½Ó¿ª·¢°åµÄVCC
#define GPIO_PIN_AF_AS_SYS                    GPIO_AF0_RTC_50Hz                // Òý½Å²»×÷Îª¸´ÓÃ¹¦ÄÜÊ¹ÓÃ

#define STEPMOTOR_ENA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOD_CLK_ENABLE()     // µç»úÍÑ»úÊ¹ÄÜ¿ØÖÆ£¬Èç¹ûÐü¿Õ²»½ÓÄ¬ÈÏÊ¹ÄÜµç»ú 

#define STEPMOTOR1_DIR_FORWARD()               HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_RESET)
#define STEPMOTOR1_DIR_REVERSAL()              HAL_GPIO_WritePin(GPIOD,GPIO_PIN_3, GPIO_PIN_SET)

#define STEPMOTOR2_DIR_FORWARD()               HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET)
#define STEPMOTOR2_DIR_REVERSAL()              HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET)

#define STEPMOTOR3_DIR_FORWARD()               HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_RESET)
#define STEPMOTOR3_DIR_REVERSAL()              HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET)

#define STEPMOTOR4_DIR_FORWARD()               HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET)
#define STEPMOTOR4_DIR_REVERSAL()              HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET)

#define STEPMOTOR1_OUTPUT_ENABLE()             HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET)
#define STEPMOTOR1_OUTPUT_DISABLE()            HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET)

#define STEPMOTOR2_OUTPUT_ENABLE()             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_RESET)
#define STEPMOTOR2_OUTPUT_DISABLE()            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_11,GPIO_PIN_SET)

#define STEPMOTOR3_OUTPUT_ENABLE()             HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_RESET)
#define STEPMOTOR3_OUTPUT_DISABLE()            HAL_GPIO_WritePin(GPIOF,GPIO_PIN_2,GPIO_PIN_SET)

#define STEPMOTOR4_OUTPUT_ENABLE()             HAL_GPIO_WritePin(GPIOH,GPIO_PIN_7,GPIO_PIN_RESET)
#define STEPMOTOR4_OUTPUT_DISABLE()            HAL_GPIO_WritePin(GPIOH,GPIO_PIN_7,GPIO_PIN_SET)
/* ------------- ???????? (??? ????? typedef Profile_t …) ------------- */
typedef struct {
    int16_t  delta_ticks;   // ??? ?????? (??? TIM8->CCR*)
    uint16_t repeat;        // ??????? ??? ?????? ?????????
		int8_t dir; 
} Profile_t;

typedef struct {
    Profile_t *pf;        /* ??????? d-tick’??                      */
    uint16_t   seg;       /* ??????? ???????                        */
    uint16_t   left;      /* ??????? ????????? ??? ? ????????       */
    uint16_t   segs_total;/* ????? ????????? ? ???????              */
		uint16_t delta_ticks_override;
} Runner_t;
extern Profile_t motor1[SEGMENTS];
extern Profile_t motor2[SEGMENTS];
extern Profile_t motor3[SEGMENTS];
/* ?????????? «???????» ? ??????? ??? ???????                  */
extern Runner_t   run[4];
extern uint16_t   max_profile_segs;
void Motor_UpdateCompare(TIM_HandleTypeDef *htim); 
void StartTrajectory(void); 
void AnalyzeProfile(Profile_t* profile, int segments, const char* label);
void BuildProfileFromAngleSegments(int16_t segments[][3], int num_segments, uint32_t total_duration_ms);
void GoToHome(void);
void Motor4_Run180Cycle(uint16_t duration_ms); 
void CalibrationTick(void);
uint8_t CalibrationRequest(void);
uint8_t MoveToWorkTopCenterRequest(void);
uint8_t EmergencyStopRequest(void);
void StartTrajectorySingleMotor(int motor_idx, int segment_count);
void AxisCounters_Init(void);
HAL_StatusTypeDef AxisCounters_Save(void);


void RefreshReadyFlag(void);
uint8_t MotionIsActive(void);
uint16_t MotionStatusFlags(void);

typedef enum { CAL_IDLE, CAL_FWD, CAL_BACK, CAL_DONE } CalibState_t;
extern CalibState_t calibState;
extern uint8_t limOK[3];

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */
