/* motor.c */
#include "motor.h"
#include "tim.h"     // ??? ???????? htim8
#include "gpio.h"    // ???? ????? ???-?? ???
#include <stdio.h>
#include "rs485.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h> 
#include "delta_steps_table.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "stm32f4xx_hal_flash_ex.h"

 
uint64_t  g_move_time             = 0;
uint16_t  g_command               = 0; 
uint32_t real_time_us = 0;       // ????????? ??????
uint8_t time_captured = 0;
uint8_t g_motorRunning[4]         = {0};  
uint8_t   ena                     = 0;
uint8_t   alarm                   = 0;
uint8_t   rs_mode                 = 0;
uint8_t   is_on                   = 0;
uint8_t   ready = 0; 
uint8_t limitSwitches[4]  = {1, 1, 1, 1};
uint8_t ready_motors[4]  = {0, 0, 0, 0};
uint32_t reverseStepCounter[4]  = {0};
uint8_t  reversingInProgress[4] = {0};
int16_t REDUCTOR_CONF = 10;  
uint32_t motor_tick[4]    = {0, 0, 0, 0};
uint8_t DEBUG = 1;
int32_t motor_step_counter[4] = {0};
volatile uint8_t motor4_busy = 0;
CalibState_t calibState = CAL_IDLE;
uint8_t limOK[3] = {0};
volatile int32_t axis_total_counter[3] = {0, 0, 0};
static int32_t axis_home_counter[3] = {0, 0, 0};
static uint8_t axis_home_valid = 0;

uint8_t MotionIsActive(void)
{
    if (calibState != CAL_IDLE)
    {
        return 1U;
    }

    for (int i = 0; i < 4; ++i)
    {
        if ((g_motorRunning[i] != 0U) || (reversingInProgress[i] != 0U))
        {
            return 1U;
        }
    }

    if (motor4_busy != 0U)
    {
        return 1U;
    }

    return 0U;
}

uint16_t MotionStatusFlags(void)
{
    uint16_t flags = 0U;

    if (g_motorRunning[0]) flags |= (1U << 0);
    if (g_motorRunning[1]) flags |= (1U << 1);
    if (g_motorRunning[2]) flags |= (1U << 2);
    if (g_motorRunning[3]) flags |= (1U << 3);

    if (reversingInProgress[0]) flags |= (1U << 4);
    if (reversingInProgress[1]) flags |= (1U << 5);
    if (reversingInProgress[2]) flags |= (1U << 6);
    if (reversingInProgress[3]) flags |= (1U << 7);

    if (calibState == CAL_FWD)  flags |= (1U << 8);
    if (calibState == CAL_BACK) flags |= (1U << 9);
    if (calibState == CAL_DONE) flags |= (1U << 10);

    if (motor4_busy) flags |= (1U << 11);

    return flags;
}

void RefreshReadyFlag(void)
{
    if (alarm != 0U)
    {
        ready = 0U;
        return;
    }

    ready = (MotionIsActive() != 0U) ? 0U : 1U;
}

#define AXIS_COUNTER_MAGIC         0x4158434EU
#define AXIS_COUNTER_VERSION       0x00000001U
#define AXIS_COUNTER_FLASH_ADDR    0x080E0000U
#define AXIS_COUNTER_FLASH_SECTOR  FLASH_SECTOR_11

typedef struct
{
    uint32_t magic;
    uint32_t version;
    int32_t axis[3];
    uint32_t crc;
} AxisCounterStorage_t;

/* ?? ?????? ??????? ?? ?????? ????? */
Profile_t motor1[240];
Profile_t motor2[240];
Profile_t motor3[240];

int16_t angle_segments[200][3] = {{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},{108, 216, 108},};
Runner_t   run[4] = {0};

// ???????? ???????????? (?????? np.linspace)
static uint32_t AxisCounter_CalcCrc(const AxisCounterStorage_t *data)
{
    const uint32_t *words = (const uint32_t *)data;
    uint32_t crc = 0xA5A5A5A5U;

    for (uint32_t i = 0; i < 5U; ++i)
    {
        crc ^= words[i];
        crc = (crc << 5) | (crc >> 27);
        crc += 0x9E3779B9U;
    }

    return crc;
}

void AxisCounters_Init(void)
{
    const AxisCounterStorage_t *stored = (const AxisCounterStorage_t *)AXIS_COUNTER_FLASH_ADDR;
    AxisCounterStorage_t copy = *stored;

    if ((copy.magic == AXIS_COUNTER_MAGIC) &&
        (copy.version == AXIS_COUNTER_VERSION) &&
        (copy.crc == AxisCounter_CalcCrc(&copy)))
    {
        axis_total_counter[0] = copy.axis[0];
        axis_total_counter[1] = copy.axis[1];
        axis_total_counter[2] = copy.axis[2];
        AddLog(0x1014);
        return;
    }

    axis_total_counter[0] = 0;
    axis_total_counter[1] = 0;
    axis_total_counter[2] = 0;
    AddLog(0x2014);
}

HAL_StatusTypeDef AxisCounters_Save(void)
{
    AxisCounterStorage_t data;
    uint32_t sectorError = 0;
    HAL_StatusTypeDef status = HAL_OK;

    data.magic = AXIS_COUNTER_MAGIC;
    data.version = AXIS_COUNTER_VERSION;

    taskENTER_CRITICAL();
    data.axis[0] = axis_total_counter[0];
    data.axis[1] = axis_total_counter[1];
    data.axis[2] = axis_total_counter[2];
    taskEXIT_CRITICAL();

    data.crc = AxisCounter_CalcCrc(&data);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef eraseInit;
    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    eraseInit.Sector = AXIS_COUNTER_FLASH_SECTOR;
    eraseInit.NbSectors = 1;

    status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
    if (status == HAL_OK)
    {
        const uint32_t *words = (const uint32_t *)&data;
        uint32_t address = AXIS_COUNTER_FLASH_ADDR;

        for (uint32_t i = 0; i < 6U; ++i)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, words[i]);
            if (status != HAL_OK)
            {
                break;
            }
            address += 4U;
        }
    }

    HAL_FLASH_Lock();

    if (status != HAL_OK)
    {
        AddLog(0x4012);
        return status;
    }

    {
        const AxisCounterStorage_t *stored = (const AxisCounterStorage_t *)AXIS_COUNTER_FLASH_ADDR;
        AxisCounterStorage_t check = *stored;

        if ((check.magic != AXIS_COUNTER_MAGIC) ||
            (check.version != AXIS_COUNTER_VERSION) ||
            (check.crc != AxisCounter_CalcCrc(&check)))
        {
            AddLog(0x4013);
            return HAL_ERROR;
        }
    }

    AddLog(0x1015);
    return HAL_OK;
}

void linspace(int16_t* arr, int start, int end, int num) {
    for (int i = 0; i < num; ++i) {
        float ratio = (float)i / (num - 1);
        arr[i] = (int16_t)(start + ratio * (end - start));
    }
}

// ????????? ??????? ????? ?? ???????? ???????????
// seg  � ????? ?????????? ?????
// acc  � ?????????? ????? ?? ??????
// dec  � ?????????? ????? ?? ??????????
// flat � ?????????? ????? ?? ?????
// ??????????: ????????? ?? ??????????? ?????? ????? seg
int16_t* get_generated_step_list(int seg, int acc, int dec, int flat, int* out_len) {
    if (acc + dec + flat != seg) {
        UART_SendString("Bad params\r\n");
				MotorStop();
        return NULL;
    }

    static int16_t delta[MAX_SEGMENT];
    if (seg > MAX_SEGMENT) {
        UART_SendString("Too many segments\r\n");
				MotorStop();
        return NULL;
    }

    linspace(delta, 300, 20, acc);
    for (int i = 0; i < flat; ++i) {
        delta[acc + i] = 20;
    }
    linspace(delta + acc + flat, 20, 300, dec);

    *out_len = seg;
    return delta;
}
void ClearMotorState()
{
    for (int i = 0; i < 3; ++i) {
        run[i].seg = 0;
        run[i].left = 0;
        g_motorRunning[i] = 0;
        reversingInProgress[i] = 0;
    }

    memset(motor1, 0, sizeof(motor1));
    memset(motor2, 0, sizeof(motor2));
    memset(motor3, 0, sizeof(motor3));
		memset(motor_step_counter, 0, sizeof(motor_step_counter));

    UART_SendString("?? Motor state reset.\r\n");
} 
void LogProfile(Profile_t *profile, uint16_t count, const char *label)
{
    char header[64];
    sprintf(header, "=== Profile [%s] ===\r\n", label);
    UART_SendString(header);

    for (uint16_t i = 0; i < count; ++i)
    {
        uint32_t duration_ms = (uint32_t)((uint64_t)profile[i].delta_ticks * profile[i].repeat * 1000ULL / TIM8_CLK_HZ);
				float freq = (profile[i].delta_ticks > 0) ? ((float)TIM8_CLK_HZ / (float)profile[i].delta_ticks) : 0.0f;
			
        char line[128];
        sprintf(line, "Segment %2d: delta=%4d  repeat=%4d  ~%4lu ms\r\n",
                i, profile[i].delta_ticks, profile[i].repeat, duration_ms);
        UART_SendString(line);
    }

    UART_SendString("========================\r\n");
		
}
void GoToHome(void) {
    UART_SendString(">> GoToHome triggered\r\n");
    
    for (int i = 0; i < 3; ++i) {
        if (motor_step_counter[i] != 0) {
            StartBackOffForMotor(i, -motor_step_counter[i]);
        }
    }
}

void build_profile(const int16_t *src, int16_t len, Profile_t *dst)
{
    const int grp = len / SEGMENTS;          /* 20 ????? */
    for (int i = 0; i < SEGMENTS; ++i) {
        uint32_t sum = 0;
        for (int k = 0; k < grp; ++k)
            sum += abs(src[i * grp + k]);
        dst[i].delta_ticks = sum / grp;                    /* ??????? */
        if (dst[i].delta_ticks == 0) dst[i].delta_ticks = 1;
        dst[i].repeat = grp;                               /* 20 ???.  */
    }
}
void StartTrajectoryFromBuiltProfiles(int motor_count, int segment_count)
{
    Profile_t* profiles[3] = { motor1, motor2, motor3 };
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    time_captured = 0U;

    for (int i = 0; i < motor_count; ++i)
    {
        run[i].pf = profiles[i];
        run[i].seg = 0;
        run[i].left = profiles[i][0].repeat;

        setMotorDirection(i, run[i].pf[0].dir);
        g_motorRunning[i] = 1;

        uint16_t first_delta = run[i].pf[0].delta_ticks;
        if (first_delta == 0) first_delta = 1;

        switch (i)
        {
            case 0:
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, first_delta);
                HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);
                break;
            case 1:
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, first_delta);
                HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_2);
                break;
            case 2:
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, first_delta);
                HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3);
                break;
            default:
                break;
        }
    }

    ready = 0U;
    UART_SendString("Trajectory started using built profiles\r\n");
} 
#define STEP_TOGGLE_FACTOR 2U
#define MAX_FREQ_STEPS_PER_SEC 64000  // ???????????? ??????? ????? (steps/sec)

void BuildProfileFromAngleSegments(int16_t segments[][3], int num_segments, uint32_t total_duration_ms)
{
    const float ticks_per_sec = (float)TIM8_CLK_HZ;
    const float time_per_segment = (total_duration_ms / 1000.0f) / num_segments;

    Profile_t* motors[3] = { motor1, motor2, motor3 };

    for (int m = 0; m < 3; ++m)
    {
        for (int i = 0; i < num_segments; ++i)
        {
            int16_t angle_min = segments[i][m];
            int32_t driver_steps = (int32_t)roundf((float)(abs(angle_min)) * STEPMOTOR_MICRO_STEP * 200.0f * REDUCTOR_CONF / 21600.0f);
            int32_t steps = driver_steps * (int32_t)STEP_TOGGLE_FACTOR;

            if (steps == 0)
            {
                motors[m][i].repeat = 0;
                motors[m][i].delta_ticks = 0;
                motors[m][i].dir = (angle_min < 0) ? 1 : 0;
                continue;
            }

            float freq = (float)steps / time_per_segment;

            // ??????? ???????? ?? ?????????? ????????
            if (freq > MAX_FREQ_STEPS_PER_SEC)
            {
                UART_SendString("? Error: Exceeded max step frequency!\r\n");
                AddLog(0x3001);  // ???????????????? ?????? ??????????
                return;
            }

            if (freq < 1.0f) freq = 1.0f;

            uint16_t delta_ticks = (uint16_t)(roundf(ticks_per_sec / freq));
            if (delta_ticks == 0) delta_ticks = 1;

            motors[m][i].delta_ticks = delta_ticks;
            motors[m][i].repeat = steps;
            motors[m][i].dir = (angle_min < 0) ? 1 : 0;
        }

        segments_loaded[m] = num_segments;
        rs_segment_count[m] = num_segments;
    }

    UART_SendString("Segmented profile with timing preservation built.\r\n");

    __HAL_TIM_SET_COUNTER(&htim5, 0);
    HAL_TIM_Base_Start(&htim5);
    ready = 0U;
    StartTrajectoryFromBuiltProfiles(3, num_segments);
}
/* ------------------------------------------------------------------ */
/*  ?????? ?????????? ?????? ??? ?????? ?????????? ?????????          */
/* ------------------------------------------------------------------ */
void StartTrajectorySingleMotor(int motor_idx, int segment_count)
{
    if (motor_idx < 0 || motor_idx > 2) return;

    Profile_t* profiles[3] = { motor1, motor2, motor3 };

    run[motor_idx].pf   = profiles[motor_idx];
    run[motor_idx].seg  = 0;
    run[motor_idx].left = profiles[motor_idx][0].repeat;

    setMotorDirection(motor_idx, run[motor_idx].pf[0].dir);
    g_motorRunning[motor_idx] = 1;

    switch (motor_idx)
    {
        case 0: HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1); break;
        case 1: HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_2); break;
        case 2: HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3); break;
    }

    ready = 0U;
    char msg[48];
    sprintf(msg, "Trajectory started (M%d, %d seg)\r\n", motor_idx + 1, segment_count);
    UART_SendString(msg);
}



int16_t CONTROL_MODE = 0;

int rotateForward(void) {
	if (!alarm) {
		STEPMOTOR1_DIR_FORWARD();
		STEPMOTOR2_DIR_FORWARD();
		STEPMOTOR3_DIR_FORWARD();
		STEPMOTOR4_DIR_FORWARD();
		sendedNotifaction=0; 
		return 0;
	}
	return 1;
}
int rotateRevers(void) {
	if (!alarm) {
		STEPMOTOR1_DIR_REVERSAL();
		STEPMOTOR2_DIR_REVERSAL();
		STEPMOTOR3_DIR_REVERSAL();
		STEPMOTOR4_DIR_REVERSAL();
		sendedNotifaction=0; 
		return 0;
	}
	return 1;
}
int MotorStopSingle(uint8_t motor_id)
{ 
    switch (motor_id)
    {
        case 0: HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_1); break;
        case 1: HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_2); break;
        case 2: HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_3); break;
        case 3: HAL_TIM_OC_Stop_IT(&htim8, TIM_CHANNEL_4); break;
    }
		reverseStepCounter[motor_id]  = 0;
		reversingInProgress[motor_id] = 0;
		g_motorRunning[motor_id]      = 0;  
    return 0;
}

int MotorStart(void) {
    if (!alarm) {
        sendedNotifaction = 0; 
            HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);
            HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_2);
            HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3);
            HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_4); 
        return 0;
    }
    return 1;
}
int MotorStop(void) { 
		HAL_TIM_OC_Stop_IT(&htim8,TIM_CHANNEL_2);
		HAL_TIM_OC_Stop_IT(&htim8,TIM_CHANNEL_1);
		HAL_TIM_OC_Stop_IT(&htim8,TIM_CHANNEL_3);
		HAL_TIM_OC_Stop_IT(&htim8,TIM_CHANNEL_4);
		return 0;
}

void AnalyzeProfile(Profile_t* profile, int segments, const char* label)
{
    uint32_t total_steps = 0;
    uint64_t total_ticks = 0;

    for (int i = 0; i < segments; ++i) {
        total_steps += profile[i].repeat;
        total_ticks += (uint64_t)profile[i].delta_ticks * profile[i].repeat;
    }

    // TIM8_CLK_HZ � ?????? ???? ?????????? ???-??, ???????? 1_000_000 (1 MHz)
    uint32_t duration_ms = (uint32_t)(total_ticks / (TIM8_CLK_HZ / 1000));

    char msg[128];
    sprintf(msg, "[%s] steps=%lu  time=�%lu ms\r\n", label, total_steps, duration_ms);
    UART_SendString(msg);
}



int MotorEnable(void) {
	if (!alarm) {
		STEPMOTOR1_OUTPUT_ENABLE(); 
		STEPMOTOR2_OUTPUT_ENABLE(); 
		STEPMOTOR3_OUTPUT_ENABLE(); 
		STEPMOTOR4_OUTPUT_ENABLE();  
		sendedNotifaction=0;
		return 0;
	}
	return 1;
}
int MotorDisable(void) { 
		STEPMOTOR1_OUTPUT_DISABLE();
		STEPMOTOR2_OUTPUT_DISABLE();
		STEPMOTOR3_OUTPUT_DISABLE();
		STEPMOTOR4_OUTPUT_DISABLE(); 
		return 0;
}  
void setMotorDirection(int motorIndex, uint8_t dir)
{
    // dir = 0 => forward, 1 => reverse
    // motorIndex = 0..3
    switch(motorIndex)
    {
        case 0:
            if (dir == 0) STEPMOTOR1_DIR_FORWARD();
            else          STEPMOTOR1_DIR_REVERSAL();
            break;
        case 1:
            if (dir == 0) STEPMOTOR2_DIR_FORWARD();
            else          STEPMOTOR2_DIR_REVERSAL();
            break;
        case 2:
            if (dir == 0) STEPMOTOR3_DIR_FORWARD();
            else          STEPMOTOR3_DIR_REVERSAL();
            break;
        case 3:
            if (dir == 0) STEPMOTOR4_DIR_FORWARD();
            else          STEPMOTOR4_DIR_REVERSAL();
            break;
        default:
            break;
    }
}

void StartBackOffForMotor(int i, int16_t steps)
{ 
    uint8_t dirBack = 0; // 0 => forward, 1 => reverse
    if (steps < 0) {
        dirBack = 1;
        steps = -steps;  // ????? ??????
    }
 
    setMotorDirection(i, dirBack);
 
    uint32_t total_steps = (uint32_t)steps * STEPMOTOR_MICRO_STEP * REDUCTOR_CONF * 200u * STEP_TOGGLE_FACTOR / 21600u;
    if (total_steps == 0) total_steps = 1; // ?? ?????? ??????

    reverseStepCounter[i]  = total_steps;
    reversingInProgress[i] = 1;
    g_motorRunning[i]      = 1; 
    switch (i)
    {
        case 0: HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1); break;
        case 1: HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_2); break;
        case 2: HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3); break;
        case 3: HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_4); break;
    }

    ready = 0U;
    UART_SendString("BackOff started motor=");
    char dbg[32];
    sprintf(dbg, "%d steps=%d dir=%d\r\n", i+1, (int)steps, (int)dirBack);
    UART_SendString(dbg);
} 


void Motor4_Run180Cycle(uint16_t duration_ms)
{
    if (motor4_busy || duration_ms == 0) return;

    uint32_t steps_180 = (uint32_t)(STEPMOTOR_MICRO_STEP * 200.0f * REDUCTOR_CONF * STEP_TOGGLE_FACTOR * 180.0f / 360.0f);
    float duration_sec = duration_ms / 1000.0f;
    float freq = steps_180 / duration_sec;

    if (freq < 1.0f) freq = 1.0f;
    if (freq > 64000.0f) freq = 64000.0f;

    uint16_t delta_ticks = (uint16_t)(TIM8_CLK_HZ / freq);
    if (delta_ticks == 0) delta_ticks = 1;

    run[3].seg = 0;
    run[3].left = steps_180;
    run[3].pf = NULL;
    motor4_busy = 1;
    g_motorRunning[3] = 1;

    setMotorDirection(3, 1); // ??????
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, __HAL_TIM_GET_COUNTER(&htim8) + delta_ticks);
    HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_4);

    // ????????? delta_ticks ??? ?????????? (???????? ? run[3].tick_time)
    run[3].delta_ticks_override = delta_ticks; // ????? ???????? ??? ???? ? Runner_t
    ready = 0U;
}




void Motor_UpdateCompare(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM8) return;

    uint8_t idx;
    uint32_t channel;

    switch (htim->Channel)
    {
        case HAL_TIM_ACTIVE_CHANNEL_1: idx = 0; channel = TIM_CHANNEL_1; break;
        case HAL_TIM_ACTIVE_CHANNEL_2: idx = 1; channel = TIM_CHANNEL_2; break;
        case HAL_TIM_ACTIVE_CHANNEL_3: idx = 2; channel = TIM_CHANNEL_3; break;
        case HAL_TIM_ACTIVE_CHANNEL_4: idx = 3; channel = TIM_CHANNEL_4; break;
        default: return;
    }

    Runner_t *r = &run[idx];

    // === ????????? ?????? 4 � ??????????? ?????? ===
    if (idx == 3)
		{
				static uint8_t going_back = 0;
				uint16_t delta_ticks = run[3].delta_ticks_override; 
				uint32_t steps_180 = (uint32_t)(STEPMOTOR_MICRO_STEP * 200.0f * REDUCTOR_CONF * 180.0f / 360.0f);

				if (r->left == 0)
				{
						if (!going_back)
						{
								going_back = 1;
								setMotorDirection(3, 0); // reverse
								r->left = steps_180;
						}
						else
						{
								going_back = 0;
                                motor4_busy = 0;
                                MotorStopSingle(3);
                                reversingInProgress[3] = 0;
                                g_motorRunning[3] = 0;
                                return;
						}
				}

				--r->left;
				motor_step_counter[3] += (going_back == 0) ? 1 : -1;

				uint32_t next = __HAL_TIM_GET_COMPARE(htim, channel) + delta_ticks;
				if (next > htim8.Init.Period)
						next -= (htim8.Init.Period + 1);

				__HAL_TIM_SET_COMPARE(htim, channel, next);
				return;
		}


    // === ????????? ??????? 1�3 ?? ??????? ===
    while (r->left == 0)
    {
        r->seg++;
        if (r->seg >= rs_segment_count[idx])
        {
            MotorStopSingle(idx );
            reversingInProgress[idx] = 0;
            g_motorRunning[idx] = 0;

            if (!time_captured &&
                (g_motorRunning[0] == 0U) &&
                (g_motorRunning[1] == 0U) &&
                (g_motorRunning[2] == 0U))
            {
                time_captured = 1;
                real_time_us = __HAL_TIM_GET_COUNTER(&htim5);
                char msg[64];
                snprintf(msg, sizeof(msg),
                         "Trajectory DONE. Real time = %lu us = %.2f ms\r\n",
                         real_time_us, real_time_us / 1000.0f);
                UART_SendString(msg);

                ClearMotorState();
                __HAL_TIM_SET_COUNTER(&htim8, 0);
                sendedNotifactionKey = 0;
            }
            return;
        }

        r->left = r->pf[r->seg].repeat;
        int16_t dt = r->pf[r->seg].delta_ticks;

        if (r->left == 0 || dt == 0)
            continue;

        setMotorDirection(idx, r->pf[r->seg].dir);
        break;
    }

    uint16_t delta = r->pf[r->seg].delta_ticks;
    if (delta == 0)
    {
        MotorStopSingle(idx);
        reversingInProgress[idx] = 0;
        g_motorRunning[idx] = 0;
        return;
    }

    --r->left;
    {
        int32_t step_delta = (r->pf[r->seg].dir == 0) ? 1 : -1;
        motor_step_counter[idx] += step_delta;
        if (idx < 3)
        {
            axis_total_counter[idx] += step_delta;
        }
    }

    uint32_t next = __HAL_TIM_GET_COMPARE(htim, channel) + delta;
    if (next > htim8.Init.Period)
        next -= (htim8.Init.Period + 1);

    __HAL_TIM_SET_COMPARE(htim, channel, next);
}

 

/* ------------------------------------------------------------------ */
/* 3.  ????? ??????????                                               */
/* ------------------------------------------------------------------ */
void StartTrajectory(void)
{ 
    /* 3.1  � ?????? ??????? ???? ??????? */
    //int out_len = 0;
		//int16_t* steps = get_generated_step_list(6400, 3200, 3200, 0, &out_len);
		//build_profile_wave_strong(10, 20, 300, 400, motor1);  
		//build_profile_wave_strong(10, 20, 300, 400, motor2);  
		//build_profile_wave_strong(10, 20, 300, 400, motor3);  
		
		AnalyzeProfile(motor1, SEGMENTS, "motor1");
		AnalyzeProfile(motor2, SEGMENTS, "motor2");
		AnalyzeProfile(motor3, SEGMENTS, "motor3");
		LogProfile(motor1, 20, "WaveByTime");

    /* 3.2  � ?????????????? ??????? */
    for (int i = 0; i < 3; ++i) {
        run[i].pf   = (i == 0) ? motor1 : (i == 1) ? motor2 : motor3;
        run[i].seg  = 0;
        run[i].left = run[i].pf[0].repeat;             /* 20 ?????????  */
    }

    /* 3.3  � ???????? ??????? ? ?????? ?????? ????????? */
    __HAL_TIM_SET_COUNTER(&htim8, 0);

    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, motor1[0].delta_ticks);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, motor2[0].delta_ticks);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, motor3[0].delta_ticks);

    /* 3.4  � ????????? ?????? */
    HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_3);
}
#define WORK_POSE_ARCMIN_FROM_HOME   -1220
#define WORK_POSE_DURATION_MS        2500U

static int32_t ArcminToToggleStepsSigned(int32_t arcmin)
{
    float driver_steps_f = (float)abs(arcmin) * (float)STEPMOTOR_MICRO_STEP * 200.0f * (float)REDUCTOR_CONF / 21600.0f;
    int32_t driver_steps = (int32_t)lroundf(driver_steps_f);
    int32_t toggles = driver_steps * (int32_t)STEP_TOGGLE_FACTOR;
    return (arcmin >= 0) ? toggles : -toggles;
}

static int32_t ToggleStepsToArcminSigned(int32_t toggles)
{
    float denom = (float)STEPMOTOR_MICRO_STEP * 200.0f * (float)REDUCTOR_CONF * (float)STEP_TOGGLE_FACTOR;
    float arcmin_f = ((float)abs(toggles) * 21600.0f) / denom;
    int32_t arcmin = (int32_t)lroundf(arcmin_f);
    return (toggles >= 0) ? arcmin : -arcmin;
}
/* motor.c  ------------------------------------------------------------ */
static void StartReleaseTrajectory(void)
{
    /* 1�10� ????? ??? ???? ???? Z-???????; 500 ?? ?? ???? ??????? */
    int16_t release_arcmin[1][3] = { { -410, -410, -410 } };
    BuildProfileFromAngleSegments(release_arcmin, 1, 1000);   /* ??????? + ????? StartTrajectory() */
}

uint8_t MoveToWorkTopCenterRequest(void)
{
    if (calibState != CAL_IDLE)
    {
        UART_SendString("[407] MoveWork: ignored (calibration busy)\r\n");
        return 0;
    }

    if (g_motorRunning[0] || g_motorRunning[1] || g_motorRunning[2])
    {
        UART_SendString("[407] MoveWork: ignored (motors busy)\r\n");
        return 0;
    }

    if (!axis_home_valid)
    {
        UART_SendString("[407] MoveWork: ignored (home not calibrated)\r\n");
        return 0;
    }

    const int32_t work_from_home_toggles = ArcminToToggleStepsSigned(WORK_POSE_ARCMIN_FROM_HOME);
    int16_t one_seg[1][3];

    for (int axis = 0; axis < 3; ++axis)
    {
        int32_t target_toggles = axis_home_counter[axis] + work_from_home_toggles;
        int32_t delta_toggles = target_toggles - axis_total_counter[axis];
        int32_t delta_arcmin = ToggleStepsToArcminSigned(delta_toggles);

        if (delta_arcmin < -32768 || delta_arcmin > 32767)
        {
            AddLog(0x3017);
            return 0;
        }

        one_seg[0][axis] = (int16_t)delta_arcmin;
    }

    if (one_seg[0][0] == 0 && one_seg[0][1] == 0 && one_seg[0][2] == 0)
    {
        AddLog(0x1018);
        return 1;
    }

    BuildProfileFromAngleSegments(one_seg, 1, WORK_POSE_DURATION_MS);
    AddLog(0x1017);
    return 1;
}

uint8_t EmergencyStopRequest(void)
{
    MotorStop();

    for (int i = 0; i < 4; ++i)
    {
        run[i].pf = NULL;
        run[i].seg = 0;
        run[i].left = 0;
        run[i].segs_total = 0;
        run[i].delta_ticks_override = 0;
        reverseStepCounter[i] = 0;
        reversingInProgress[i] = 0;
        g_motorRunning[i] = 0;
        ready_motors[i] = 0;
    }

    motor4_busy = 0;
    calibState = CAL_IDLE;
    time_captured = 0;
    memset(motor_step_counter, 0, sizeof(motor_step_counter));

    RefreshReadyFlag();
    UART_SendString("[303] Hard stop executed\r\n");
    return 1;
}

uint8_t CalibrationRequest(void)                    /* motor.c */
{
    if (calibState != CAL_IDLE)
    {
        UART_SendString("[406] Calib: ignored (busy)\r\n");
        return 0;
    }

    int16_t oneTurn[5][3] = {
        { 3500*3, 3500*3, 3500*3 },
        { 3500*3, 3500*3, 3500*3 },
        { 3500*3, 3500*3, 3500*3 },
        { 3500*3, 3500*3, 3500*3 },
        { 3500*3, 3500*3, 3500*3 }
    };
    BuildProfileFromAngleSegments(oneTurn, 1, 20000);

    memset(limOK, 0, sizeof(limOK));
    axis_home_valid = 0;
    time_captured = 0;
    calibState = CAL_FWD;
    ready = 0U;
    UART_SendString("[406] Calib: 360-deg trajectory start\r\n");
    return 1;
}
void CalibrationTick(void)                          /* motor.c */
{
    switch (calibState)
    {
        case CAL_FWD:
            for (int i = 0; i < 3; ++i)
                if (!limOK[i] && limitSwitches[i] == 0)   /* 0 = ?????? */
                {
                    MotorStopSingle(i);              /* ???? ?????? ??? */
                    limOK[i] = 1;
                }

            if (limOK[0] && limOK[1] && limOK[2])
            {
                StartReleaseTrajectory();
								calibState = CAL_BACK;
            }
            break;

        case CAL_BACK:
            if (g_motorRunning[0] == 0 && g_motorRunning[1] == 0 && g_motorRunning[2] == 0)
            {
                for (int i = 0; i < 3; ++i)
                {
                    axis_home_counter[i] = axis_total_counter[i];
                }
                axis_home_valid = 1;

                if (AxisCounters_Save() == HAL_OK)
                {
                    AddLog(0x1016);
                }
                else
                {
                    AddLog(0x2016);
                }
                calibState = CAL_DONE;
                UART_SendString("[406] Calib: done\r\n");
                AddLog(0x1013);
            }
            break;

        case CAL_DONE:
            calibState = CAL_IDLE;
            break;

        default:
            break;
    }
}

void Motor4_Run90Cycle(uint16_t duration_ms)
{
    if (motor4_busy || duration_ms == 0) return;

    uint32_t steps_90 = (uint32_t)(STEPMOTOR_MICRO_STEP * 200.0f * REDUCTOR_CONF * STEP_TOGGLE_FACTOR * 90.0f / 360.0f);
    float duration_sec = duration_ms / 1000.0f;
    float freq = steps_90 / duration_sec;

    if (freq < 1.0f) freq = 1.0f;
    if (freq > 64000.0f) freq = 64000.0f;

    uint16_t delta_ticks = (uint16_t)(roundf(TIM8_CLK_HZ / freq));
    if (delta_ticks == 0) delta_ticks = 1;

    Profile_t p;
    p.delta_ticks = delta_ticks;
    p.repeat = steps_90;
    p.dir = 0;

    run[3].pf = &p;
    run[3].seg = 0;
    run[3].left = steps_90;
    motor4_busy = 1;

    setMotorDirection(3, 0);
    HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_4);
}
 
uint8_t ch;

int fputc(int ch, FILE *f)
{ 
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_SET);
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int fgetc(FILE * f)
{
  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);
  while(HAL_UART_Receive(&huart3,&ch, 1, 0xffff)!=HAL_OK);
  return ch;
}
/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
