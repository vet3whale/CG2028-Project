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



/*--------------------------- Helper Functions ------------------------------*/
static inline float absf(float x) {
    return (x < 0.0f) ? -x : x;
}

extern float sqrtf_approx(float x);

/*--------------------------- Types -----------------------------------------*/

typedef enum {
    FALL_NORMAL,
    FALL_SUSPECT,
    FALL_CONFIRMED
} fall_state_t;

/*--------------------------- Constants ---------------------------------------*/
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
#define TRIGGER_A_MIN 12.0f

// LED Blink Behaviour

// LED blink half-periods
#define NORMAL_TOGGLE_MS 500
#define FAST_TOGGLE_MS 100
// how long to blink fast after a fall
#define FAST_BLINK_HOLD_MS 3000

// End Loop Behaviour
// boolean to see if should stop the loop
#define END_AFTER_MS 4000


/*--------------------------- Module State ---------------------------------------*/
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
static char prev_sign = '+';
static uint32_t axis_candidate_ms = 0;
static int axis_candidate = -1;
static uint32_t last_orient_trigger_ms = 0;

// fall/trigger latching (Memory of a recent impact)
static uint8_t impact_latched = 0;
static uint32_t impact_time_ms = 0;
static uint8_t latched_rot = 0;
static uint8_t latched_jerk = 0;
static float latched_a_mag = 0.0f;

/*--------------------------- Forward Declarations ---------------------------------------*/
static void UART1_Init(void);
extern void initialise_monitor_handles(void);
extern int mov_avg(int N, int* accel_buff);
static void process_axis_transition(uint32_t now, int top1_axis, char current_sign);

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

    float a_mag, g_mag, jerk = 0;

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

        // gyro "velocity"
        float gyro_velocity[3] = {0.0f};
        gyro_velocity[0] = gyro_data[0] / 1000.0f;
        gyro_velocity[1] = gyro_data[1] / 1000.0f;
        gyro_velocity[2] = gyro_data[2] / 1000.0f;

        // filtered accel (asm)
        float accel_filt_asm[3] = {0};
        accel_filt_asm[0] = (float)mov_avg(N, accel_buff_x) * (9.8f / 1000.0f);
        accel_filt_asm[1] = (float)mov_avg(N, accel_buff_y) * (9.8f / 1000.0f);
        accel_filt_asm[2] = (float)mov_avg(N, accel_buff_z) * (9.8f / 1000.0f);


        /***************************UART transmission*******************************************/
        char buffer[150];
        uint32_t now = HAL_GetTick();

        if (!freeze_uart && i >= 3 && (now - last_uart_print_ms) >= UART_PRINT_PERIOD_MS)
        {
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

            sprintf(buffer, "a_mag : %f; g_mag : %f; jerk : %f\r\n\n",
            		a_mag, g_mag, jerk);
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            last_uart_print_ms = now;
        }

        // ********* Fall detection: free-fall -> confirm by impact/rotation/orientation-change *********

        // ********* Rotation/orientation trigger: dominant axis switch (X<->Y<->Z) *********

        float ax = accel_filt_asm[0];
        float ay = accel_filt_asm[1];
        float az = accel_filt_asm[2];

        a_mag = sqrtf_approx(ax*ax + ay*ay + az*az);

        float gx = gyro_data[0];
        float gy = gyro_data[1];
        float gz = gyro_data[2];
        g_mag = sqrtf_approx(gx*gx + gy*gy + gz*gz);

        // ===== Sudden motion detection =====
        // Gyro magnitude already computed as g_mag

        uint8_t sudden_rot = (g_mag >= SUDDEN_GYRO_DPS_MIN);

        if (!prev_a_mag_valid) {
            prev_a_mag = a_mag;
            prev_a_mag_valid = 1;
        } else {
            jerk = absf(a_mag - prev_a_mag) / (SAMPLE_PERIOD_MS / 1000.0f);
            prev_a_mag = a_mag;
        }

        uint8_t sudden_jerk = (jerk >= SUDDEN_JERK_MIN);
        uint8_t sudden_action = (sudden_rot || sudden_jerk);

        if (sudden_action && a_mag >= TRIGGER_A_MIN) {
			impact_latched = 1;
			impact_time_ms = now;
			latched_rot = sudden_rot;
			latched_jerk = sudden_jerk;
			latched_a_mag = a_mag;
		}

		// expire the latch if the board doesnt settle into a new axis within 1.5 seconds
		if (impact_latched && (now - impact_time_ms) > 1500) {
			impact_latched = 0;
		}

        // only consider orientation when magnitude is near 1g
        uint8_t stable_1g = (a_mag >= G_STABLE_MIN && a_mag <= G_STABLE_MAX);

        float abx = absf(ax), aby = absf(ay), abz = absf(az);
        // find top 2 strongest axis
        int top1_axis = 0;
        float top1 = abx;
        float top2 = 0.0f;

        if (aby > top1) { top2 = top1; top1 = aby; top1_axis = 1; }
        else { top2 = aby; }

        if (abz > top1) { top2 = top1; top1 = abz; top1_axis = 2; }
        else if (abz > top2) { top2 = abz; }

        // dominance + margin checks
        uint8_t dom_ok = (top1 / a_mag) >= DOM_AXIS_FRAC_MIN;
        uint8_t margin_ok = ((top1 - top2) / a_mag) >= AXIS_SWITCH_MARGIN_FRAC;

        uint8_t axis_valid_now = stable_1g && dom_ok && margin_ok;

        if (axis_valid_now)
        {
            float raw_vals[] = {ax, ay, az};
            char current_sign = (raw_vals[top1_axis] >= 0) ? '+' : '-';
        	if (!prev_axis_valid) // runs once at init
            {
                prev_axis = top1_axis;
                prev_sign = current_sign;
                prev_axis_valid = 1;
                axis_candidate = top1_axis;
                axis_candidate_ms = now;
            }
            else
            {
                // if axis differs, check new candidate + whether sudden action
                if (top1_axis != prev_axis)
                {
                    if (axis_candidate != top1_axis)
                    {
                        axis_candidate = top1_axis;
                        axis_candidate_ms = now;
                    }
                    process_axis_transition(now, top1_axis, current_sign);
                }
                // same axis, keep candidate in sync
                else
                {
                    axis_candidate = top1_axis;
                    axis_candidate_ms = now;
                }
            }
        }
        if (fall_state == FALL_SUSPECT) {
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
        uint32_t toggle_ms = (fall_state == FALL_SUSPECT) ? FAST_TOGGLE_MS : NORMAL_TOGGLE_MS;

        if ((now - led_last_toggle_ms) >= toggle_ms) {
            BSP_LED_Toggle(LED2);
            led_last_toggle_ms = now;
        }

        HAL_Delay(SAMPLE_PERIOD_MS);
        i++;

    }
}

// confirms axis transition if it persists long enough AND cooldown passed
static void process_axis_transition(uint32_t now, int top1_axis, char current_sign)
{
    // check how long new axis has been stable
    if ((now - axis_candidate_ms) >= ORIENT_DEBOUNCE_MS)
    {
        // SCENARIO A: Violent Transition (Checks the LATCH, not current values)
        if (impact_latched && (now - last_orient_trigger_ms) >= ORIENT_COOLDOWN_MS)
        {
            char axes_names[] = {'X', 'Y', 'Z'};
            char buffer[150];

            sprintf(buffer, "\r\n!!! FALL DETECTED !!!\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            // Use the LATCHED variables to see what caused the fall
            if (latched_rot && latched_jerk) {
                sprintf(buffer, "TRIGGER  : ROT + JERK\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
                sprintf(buffer, "Direction: %c%c -> %c%c | Acc: %.2f g\r\n",
                        prev_sign, axes_names[prev_axis], current_sign, axes_names[top1_axis], latched_a_mag);
            }
            else if (latched_rot) {
                sprintf(buffer, "TRIGGER  : SUDDEN ROTATION\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
                sprintf(buffer, "Direction: %c%c -> %c%c\r\n",
                        prev_sign, axes_names[prev_axis], current_sign, axes_names[top1_axis]);
            }
            else {
                sprintf(buffer, "TRIGGER  : SUDDEN JERK\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
                sprintf(buffer, "Acc Change: %.2f g (Impact)\r\n", latched_a_mag);
            }
            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

            // Reset the latch so it doesn't double-trigger
            impact_latched = 0;

            last_orient_trigger_ms = now;
            fall_state = FALL_SUSPECT;
            t_confirm_ms = now;
            freeze_uart = 1;
        }

        // SCENARIO B: Peaceful Transition
        prev_axis = top1_axis;
        prev_sign = current_sign;
    }
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
