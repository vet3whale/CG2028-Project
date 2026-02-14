/******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * (c) CG2028 Teaching Team
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include <math.h>
#include "stdio.h"
#include "string.h"
#include <sys/stat.h>

#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"



// ====================== HELPER FOR FALL DETECTION =========================
static inline float absf(float x) {
    return (x < 0.0f) ? -x : x;
}

static inline float sqrtf_approx(float x) {
    return sqrtf(x);
}

//======================= TYPES =========================

typedef enum {
    FALL_NORMAL = 0,
    FALL_SUSPECT,
    FALL_CONFIRMED
} fall_state_t;

// ======================= CONSTANTS =========================
// Gravity
#define G_NORM 9.8f

// sampling/timing
#define SAMPLE_PERIOD_MS 20
#define UART_PRINT_PERIOD_MS 300

// orientation detection (gravity/dominant axis)
#define G_STABLE_MIN 7.0f
#define G_STABLE_MAX 12.5f
#define DOM_AXIS_FRAC_MIN 0.55f
#define AXIS_SWITCH_MARGIN_FRAC 0.10f
#define ORIENT_DEBOUNCE_MS 150
#define ORIENT_COOLDOWN_MS 400


// "sudden motion" gates
//determines how fast the rotation needs to be (the higher it is, the faster it needs to be before trigger)
#define SUDDEN_GYRO_DPS_MIN 220.0f

// determines how fast acceleration magnitude must change before allow trigger
#define SUDDEN_JERK_MIN 2.5f

// Trigger Thresholds
// the acceleration it needs to trigger fall
// must be above 13.5, and trigger around 13.5
#define TRIGGER_A_MIN 12.0f

// optional legacy fall thresholds
#define FREEFALL_A_MAG_MAX 3.0f     // m/s^2  (~0.3g)
#define FREEFALL_MIN_MS 120
#define IMPACT_A_MAG_MIN 15.0f      // m/s^2  (~1.5g)
#define SUSPECT_TIMEOUT_MS 1500
// rotation trigger (extra robustness)
#define GYRO_MAG_THR 180.0f

// LED Blink Behaviour

// LED blink half-periods
#define NORMAL_TOGGLE_MS 500
#define FAST_TOGGLE_MS 100
// how long to blink fast after a fall
#define FAST_BLINK_HOLD_MS 3000

// End Loop Behaviour
// boolean to see if should stop the loop
#define END_AFTER_MS 4000


// ================= MODULE STATE ===================
// fall/trigger state
static fall_state_t fall_state = FALL_NORMAL;
static uint32_t t_confirm_ms = 0;

// uart control
static uint32_t last_uart_print_ms = 0;
static uint8_t freeze_uart = 0;

// jerk calculation
static float prev_a_mag = 0.0f;
static uint8_t prev_a_mag_valid = 0;

// loop stopping
static uint8_t stop_loop = 0;
static uint8_t endloop_printed = 0;

// led timing
static uint32_t led_last_toggle_ms = 0;

// axis/orientation tracking
static int prev_axis = -1;
static uint8_t prev_axis_valid = 0;
static uint32_t axis_candidate_ms = 0;
static int axis_candidate = -1;
static uint32_t last_orient_trigger_ms = 0;

// ================== FORWARD DECLARATIONS ========================
static void UART1_Init(void);
extern void initialise_monitor_handles(void);
extern int mov_avg(int N, int* accel_buff);
int mov_avg_C(int N, int* accel_buff);

// ============== UART Peripherals =========================
UART_HandleTypeDef huart1;

int main(void)
{
    const int N = 4;

    HAL_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();

    BSP_LED_Off(LED2);

    int accel_buff_x[4] = {0};
    int accel_buff_y[4] = {0};
    int accel_buff_z[4] = {0};

    int i = 0;

    while (!stop_loop)
    {
        int16_t accel_data_i16[3] = {0};
        BSP_ACCELERO_AccGetXYZ(accel_data_i16);

        // circular buffers
        accel_buff_x[i % 4] = accel_data_i16[0];
        accel_buff_y[i % 4] = accel_data_i16[1];
        accel_buff_z[i % 4] = accel_data_i16[2];

        // gyro
        float gyro_data[3] = {0.0f};
        float* ptr_gyro = gyro_data;
        BSP_GYRO_GetXYZ(ptr_gyro);

        // gyro "velocity" (keeping your math)
        float gyro_velocity[3] = {0.0f};
        gyro_velocity[0] = (gyro_data[0] * 9.8f / 1000.0f);
        gyro_velocity[1] = (gyro_data[1] * 9.8f / 1000.0f);
        gyro_velocity[2] = (gyro_data[2] * 9.8f / 1000.0f);

        // filtered accel (asm)
        float accel_filt_asm[3] = {0};
        accel_filt_asm[0] = (float)mov_avg(N, accel_buff_x) * (9.8f / 1000.0f);
        accel_filt_asm[1] = (float)mov_avg(N, accel_buff_y) * (9.8f / 1000.0f);
        accel_filt_asm[2] = (float)mov_avg(N, accel_buff_z) * (9.8f / 1000.0f);

        // filtered accel (C)
        float accel_filt_c[3] = {0};
        accel_filt_c[0] = (float)mov_avg_C(N, accel_buff_x) * (9.8f / 1000.0f);
        accel_filt_c[1] = (float)mov_avg_C(N, accel_buff_y) * (9.8f / 1000.0f);
        accel_filt_c[2] = (float)mov_avg_C(N, accel_buff_z) * (9.8f / 1000.0f);

        /***************************UART transmission*******************************************/
        char buffer[150];
        uint32_t now = HAL_GetTick();

        if (!freeze_uart && i >= 3 && (now - last_uart_print_ms) >= UART_PRINT_PERIOD_MS)
        {
            sprintf(buffer, "Results of C execution for filtered accelerometer readings:\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
                    accel_filt_c[0], accel_filt_c[1], accel_filt_c[2]);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            sprintf(buffer, "Results of assembly execution for filtered accelerometer readings:\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
                    accel_filt_asm[0], accel_filt_asm[1], accel_filt_asm[2]);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            sprintf(buffer, "Gyroscope sensor readings:\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n\n",
                    gyro_velocity[0], gyro_velocity[1], gyro_velocity[2]);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            last_uart_print_ms = now;
        }

        // ********* Fall detection: free-fall -> confirm by impact/rotation/orientation-change *********

        // ********* Rotation/orientation trigger: dominant axis switch (X<->Y<->Z) *********

        float ax = accel_filt_c[0];
        float ay = accel_filt_c[1];
        float az = accel_filt_c[2];

        float a_mag = sqrtf_approx(ax*ax + ay*ay + az*az);

        float gx = gyro_data[0];
        float gy = gyro_data[1];
        float gz = gyro_data[2];
        float g_mag = sqrtf_approx(gx*gx + gy*gy + gz*gz);

        // ===== Sudden motion detection =====
        // Gyro magnitude already computed as g_mag

        uint8_t sudden_rot = (g_mag >= SUDDEN_GYRO_DPS_MIN);

        float jerk = 0.0f;
        if (!prev_a_mag_valid) {
            prev_a_mag = a_mag;
            prev_a_mag_valid = 1;
        } else {
            jerk = absf(a_mag - prev_a_mag) / (SAMPLE_PERIOD_MS / 1000.0f);
            prev_a_mag = a_mag;
        }

        uint8_t sudden_jerk = (jerk >= SUDDEN_JERK_MIN);
        uint8_t sudden_action = (sudden_rot || sudden_jerk);

        // only consider orientation when magnitude is near 1g
        uint8_t stable_1g = (a_mag >= G_STABLE_MIN && a_mag <= G_STABLE_MAX);

        float abx = absf(ax), aby = absf(ay), abz = absf(az);

        // find top1 axis and top2 value (for hysteresis)
        int top1_axis = 0;
        float top1 = abx;
        float top2 = 0.0f;

        if (aby > top1) { top2 = top1; top1 = aby; top1_axis = 1; }
        else { top2 = aby; }

        if (abz > top1) { top2 = top1; top1 = abz; top1_axis = 2; }
        else if (abz > top2) { top2 = abz; }

        // dominance + margin checks
        uint8_t dom_ok = (a_mag > 0.001f) && ((top1 / a_mag) >= DOM_AXIS_FRAC_MIN);
        uint8_t margin_ok = (a_mag > 0.001f) && (((top1 - top2) / a_mag) >= AXIS_SWITCH_MARGIN_FRAC);

        uint8_t axis_valid_now = stable_1g && dom_ok && margin_ok;

        if (axis_valid_now)
        {
            if (!prev_axis_valid)
            {
                prev_axis = top1_axis;
                prev_axis_valid = 1;
                axis_candidate = top1_axis;
                axis_candidate_ms = now;
            }
            else
            {
                // if axis differs, start / continue debounce candidate
                if (top1_axis != prev_axis)
                {
                    if (axis_candidate != top1_axis)
                    {
                        axis_candidate = top1_axis;
                        axis_candidate_ms = now;
                    }

                    // confirm switch if it persists long enough AND cooldown passed
                    if (sudden_action &&
						a_mag >= TRIGGER_A_MIN &&
                        (now - axis_candidate_ms) >= ORIENT_DEBOUNCE_MS &&
                        (now - last_orient_trigger_ms) >= ORIENT_COOLDOWN_MS)

                    {
                        int from_axis = prev_axis;
                        int to_axis = top1_axis;

                        prev_axis = top1_axis;
                        last_orient_trigger_ms = now;

                        // trigger indication (reuse your CONFIRMED blink)
                        fall_state = FALL_CONFIRMED;
                        t_confirm_ms = now;

                        led_last_toggle_ms = now;
                        BSP_LED_Off(LED2);

                        // single UART line (same style as before; not touching your big UART block)
                        sprintf(buffer, "FALL DETECTED!! axis=%d->%d a=%.2f\r\n", from_axis, to_axis, a_mag);
                        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

                        freeze_uart = 1;
                    }
                }
                else
                {
                    // same axis, keep candidate in sync
                    axis_candidate = top1_axis;
                    axis_candidate_ms = now;
                }
            }
        }
        if (fall_state == FALL_CONFIRMED) {
            if ((now - t_confirm_ms) >= FAST_BLINK_HOLD_MS) {

                // print END LOOP once
                if (!endloop_printed) {
                    sprintf(buffer, "END LOOP\r\n");
                    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
                    endloop_printed = 1;
                }

                // end the while loop
                stop_loop = 1;
            }
        }

        // LED timing restored (critical)
        // LED blink timing based on state, BUT keep sensor sampling fast
        uint32_t toggle_ms = (fall_state == FALL_CONFIRMED) ? FAST_TOGGLE_MS : NORMAL_TOGGLE_MS;

        if ((now - led_last_toggle_ms) >= toggle_ms) {
            BSP_LED_Toggle(LED2);
            led_last_toggle_ms = now;
        }

        HAL_Delay(SAMPLE_PERIOD_MS);
        i++;

    }
}

// ====================== MOVING AVERAGE C ======================
int mov_avg_C(int N, int* accel_buff)
{
    int result = 0;
    for (int i = 0; i < N; i++)
    {
        result += accel_buff[i];
    }
    result = result / N;
    return result;
}

// ====================== UART INIT ======================
static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        while (1) {}
    }
}

// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
