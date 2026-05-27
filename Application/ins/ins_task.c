/**
 * @file    ins_task.c
 * @brief   Application灞?鈥?INS 鎯€у鑸换鍔″疄鐜? * @note    鏁村悎 BMI088(Device) + Mahony(Algorithm) + Bluetooth(Device)
 *          鍙傝€?QGimbal: SPI DMA + EXTI DRDY 涓柇椹卞姩 + TaskNotify 鍚屾
 *
 *          杩愯妯″紡:
 *          1. 鍒濆鍖栭樁娈? SPI 杞璇诲啓 BMI088 瀵勫瓨鍣?+ 5s闆堕鏍″噯
 *          2. 杩愯闃舵:   EXTI DRDY 鈫?SPI DMA 璇诲彇 鈫?TaskNotify 鈫?Mahony
 *
 *          閬靛惊 Rule 2: 姝ゆ枃浠朵笉鍖呭惈浠讳綍 MCU HAL 澶存枃浠? *          閫氳繃 Device 灞?Register 鏈哄埗娉ㄥ叆 BSP 鍑芥暟
 */

#include "ins_task.h"
#include "main.h"
#include <math.h>

/* Device 灞?*/
#include "bmi088.h"
#include "bmi088_reg.h"
#include "bluetooth.h"
#include "../../Device/laser/laser.h"
#include "vision.h"

/* Algorithm 灞?*/
#include "attitude_algorithm.h"
#include "bsp_can.h"
#include "config.h"
#include "gimbal_task.h"
#include "MODE_GIMBAL_STAB.h"
#include "MODE_VISION_STAB.h"
#include "MODE_ANGLE_TEST.h"
#include "MODE_GRAVITY_CALIB.h"
#include "MODE_TORQUE_TEST.h"

/* BSP 灞?(浠呯敤浜?Init 鏃舵敞鍏ュ嚱鏁版寚閽? */
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "bsp_uart6.h"
#include "bsp_delay.h"
#include "bsp_laser.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <string.h>

/* ======================== 璺ㄤ换鍔℃毚闇茬殑鍏ㄥ眬鍙橀噺 ======================== */
float global_gimbal_yaw = 0.0f;
float global_gimbal_pitch = 0.0f;
float global_imu_yaw_speed = 0.0f;
float global_imu_pitch_speed = 0.0f;

/* ======================== SPI DMA 缂撳啿鍖?(鍙傝€?QGimbal) ======================== */

#define SPI_DMA_GYRO_LEN      8   /* 1(cmd) + 6(data) + 1(dummy) */
#define SPI_DMA_ACCEL_LEN     9   /* 1(cmd) + 1(dummy) + 6(data) + 1 */
#define SPI_DMA_TEMP_LEN      4   /* 1(cmd) + 1(dummy) + 2(data) */

#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

static uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LEN];
static uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LEN]  = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LEN];
static uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LEN] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t temp_dma_rx_buf[SPI_DMA_TEMP_LEN];
static uint8_t temp_dma_tx_buf[SPI_DMA_TEMP_LEN]   = {0xA2, 0xFF, 0xFF, 0xFF};

/* ======================== DMA 鐘舵€佹爣蹇?(鍙傝€?QGimbal) ======================== */

#define IMU_DR_SHFITS       0
#define IMU_SPI_SHFITS      1
#define IMU_UPDATE_SHFITS   2

static volatile uint8_t gyro_update_flag = 0;
static volatile uint8_t accel_update_flag = 0;
static volatile uint8_t accel_temp_update_flag = 0;
static volatile uint8_t imu_start_dma_flag = 0;

/* ======================== 妯″潡瀹炰緥 ======================== */

static BMI088_Instance_t    g_bmi088;
static Bluetooth_Instance_t g_bluetooth;
static Laser_Instance_t     g_laser;

/* ======================== Device 瀹炰緥 ======================== */
static Vision_Instance_t g_vision;

/* INS 浠诲姟鍙ユ焺 (鐢ㄤ簬 TaskNotify) */
static TaskHandle_t ins_task_handle_local = NULL;

/* IMU 鏁版嵁 (DMA 瑙ｆ瀽鍚? */
static float imu_gyro[3]  = {0};
static float imu_accel[3] = {0};
static float imu_temp     = 0;

/* Yaw 杩愯鏃舵紓绉昏ˉ鍋?(鍙傝€?QGimbal) - 鐩墠宸蹭娇鐢ㄩ潤鎬佽ˉ鍋?*/
// static float yaw_runtime_bias = 0.0f;

/* ======================== 鍓嶅悜澹版槑 ======================== */

static void imu_cmd_spi_dma(void);
static void gyro_read_over(const uint8_t *rx_buf, float gyro_out[3]);
static void accel_read_over(const uint8_t *rx_buf, float accel_out[3]);
static void temperature_read_over(const uint8_t *rx_buf, float *temp_out);
static uint8_t parse_signed_float(const char *text, uint16_t len, uint16_t start, float *out);

static float pitch_control_angle(float enc_deg)
{
    if (enc_deg > 180.0f) enc_deg -= 360.0f;
    return enc_deg;
}

/* ======================== BSP <-> Device 绮樺悎 ======================== */

static bool bt_send_adapter(const uint8_t *data, uint16_t len)
{
    return BSP_UART1_SendBytes(data, len);
}

/* 鎶?BSP_UART1 鏀跺埌鐨勫瓧鑺傚杺缁?Bluetooth Device */
static void bt_byte_dispatcher(uint8_t b)
{
    Bluetooth_FeedRxByte(&g_bluetooth, b);
}

/* 钃濈墮鏀跺埌涓€琛?(BSP鈫払luetooth 瑁呴厤鍚庡洖璋冨埌杩欓噷, ISR 涓婁笅鏂?
 * 鍙仛杞婚噺绾у瓧娈垫彁鍙? 涓嶅仛 snprintf / 闃诲 API */
static uint8_t parse_signed_float(const char *text, uint16_t len, uint16_t start, float *out)
{
    float val = 0.0f;
    int sign = 1;
    uint16_t j = start;
    uint8_t has_num = 0;

    while (j < len && text[j] == ' ') j++;
    if (j < len && text[j] == '-') { sign = -1; j++; }
    else if (j < len && text[j] == '+') { j++; }

    while (j < len && text[j] >= '0' && text[j] <= '9') {
        val = val * 10.0f + (text[j] - '0');
        has_num = 1;
        j++;
    }
    if (j < len && text[j] == '.') {
        float frac = 10.0f;
        j++;
        while (j < len && text[j] >= '0' && text[j] <= '9') {
            val += (text[j] - '0') / frac;
            frac *= 10.0f;
            has_num = 1;
            j++;
        }
    }

    if (!has_num) return 0;
    *out = val * sign;
    return 1;
}

static void set_external_yaw_target(float yaw)
{
    ext_target_yaw = yaw;
    flag_ext_yaw   = 1;
}

static void bt_line_callback(const char *line, uint16_t len)
{
    if (line[0] == 'Y' || line[0] == 'y') {
        float val = 0.0f;
        if (parse_signed_float(line, len, 1U, &val))
            set_external_yaw_target(val);
    }
}

static void bt_cmd_callback(uint8_t cmd_id, const uint8_t *payload, uint8_t len)
{
    switch (cmd_id)
    {
    case BT_CMD_REMOTE:
        if (len >= 1 && payload[0] == 0x01)
        {
            BMI088_CalibrateZeroDrift(&g_bmi088);
        }
        break;
    default:
        break;
    }
}

/* ======================== 鍒濆鍖?======================== */

void INS_Init(void)
{
    /* ----- 1. BSP 鍒濆鍖?----- */
    BSP_SPI_Init();
    BSP_UART1_Init();
    BSP_UART6_Init();
    BSP_Laser_Init();

    /* ----- 1.1. Device 瀹炰緥鍖? 娉ㄥ叆 BSP 鎺ュ彛 -----
     *   App 灞傛妸 BSP 鐨?send/register 鍑芥暟鎸囬拡浜ょ粰 Device 妯″潡, Device 妯″潡
     *   鍚庣画涓嶅啀鐩存帴渚濊禆 BSP, 婊¤冻 ti.md Rule 2/3 (璺ㄥ眰瑙ｈ€?+ 娉ㄥ唽鏈哄埗) */
    Bluetooth_Init(&g_bluetooth, bt_send_adapter);
    Bluetooth_RegisterRxCallback(&g_bluetooth, bt_cmd_callback);
    Bluetooth_RegisterLineCallback(&g_bluetooth, bt_line_callback);
    BSP_UART1_RegisterRxCallback(bt_byte_dispatcher);

    Vision_Init(&g_vision, BSP_UART6_SendBytes, BSP_UART6_RegisterRxCallback);
    Laser_Config_t laser_config = {
        .set_output = BSP_Laser_Set,
        .get_output = BSP_Laser_GetState,
    };
    Laser_Init(&g_laser, &laser_config);
    Laser_SetOn(&g_laser, 1U);

    /* ----- 2. BMI088 鍒濆鍖?(杞妯″紡锛屾敞鍏?BSP 鍑芥暟鎸囬拡) ----- */
    BMI088_Config_t bmi_config = {
        .spi_read_write = BSP_SPI_ReadWriteByte,
        .accel_cs_low   = BSP_SPI_AccelCS_Low,
        .accel_cs_high  = BSP_SPI_AccelCS_High,
        .gyro_cs_low    = BSP_SPI_GyroCS_Low,
        .gyro_cs_high   = BSP_SPI_GyroCS_High,
        .delay_us       = BSP_Delay_Us,
        .delay_ms       = BSP_Delay_Ms,
    };

    while (BMI088_Init(&g_bmi088, &bmi_config) != 0)
    {
        BSP_Delay_Ms(100);
    }

    /* ----- 3. 闆堕鏍″噯 (宸叉敼涓洪潤鎬佽ˉ鍋匡紝璺宠繃闃诲 5 绉? ----- */
    // BMI088_CalibrateZeroDrift(&g_bmi088);
    // g_bmi088.calib_start_tick = xTaskGetTickCount();

    /* ----- 4. Mahony 婊ゆ尝鍣ㄥ垵濮嬪寲 ----- */
    Filter_Init();

    /* ----- 5. 鍒囨崲鍒?SPI DMA 妯″紡 (杩愯鏃堕珮閫熻鍙? ----- */
    BSP_SPI_DMA_Init();
}

/* ======================== INS 涓讳换鍔?======================== */

/**
 * @brief  INS FreeRTOS 浠诲姟 鈥?鐢?DRDY 涓柇 + SPI DMA 椹卞姩
 * @note   鍙傝€?QGimbal InsTask:
 *         DRDY EXTI 鈫?SPI DMA 浼犺緭 鈫?DMA瀹屾垚涓柇 鈫?TaskNotify 鈫?濮挎€佽В绠? *         瀹為檯棰戠巼鐢?BMI088 DRDY 鍐冲畾 (~1kHz for gyro)
 *         濡?DRDY 鏈帴绾匡紝鍒欏洖閫€鍒?vTaskDelayUntil 瀹氭椂 200Hz 杞
 */
void INS_Task(void *argument)
{
    (void)argument;

    /* 淇濆瓨浠诲姟鍙ユ焺鐢ㄤ簬 ISR 涓殑 TaskNotify */
    ins_task_handle_local = xTaskGetCurrentTaskHandle();

    /* 鍚姩 DMA 妯″紡 */
    imu_start_dma_flag = 1;

    uint32_t init_counter = 0; // 鏇挎崲 TickType_t锛屼娇鐢ㄦ槑纭殑寰幆璁℃暟
    float yaw_offset = 0.0f, pitch_offset = 0.0f, roll_offset = 0.0f;
    uint32_t vision_last_tx_tick = 0U;

    for (;;)
    {
        /* 绛夊緟 SPI DMA 瀹屾垚閫氱煡 (濡傛湁 DRDY 鎺ョ嚎)
         * 瓒呮椂 10ms 浣滀负鍥為€€: 鍗充娇鏃?DRDY 涓柇涔熻兘浠?100Hz 杩愯 */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));

        static float gyro_bias[3] = {0.000902f, 0.003273f, -0.001428f};
        static float gyro_sum[3] = {0.0f, 0.0f, 0.0f};

        /* Process completed gyro DMA data. */
        if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, imu_gyro);

            if (gimbal_state == GIMBAL_STATE_CALIBRATING) {
                gyro_sum[0] += imu_gyro[0];
                gyro_sum[1] += imu_gyro[1];
                gyro_sum[2] += imu_gyro[2];
            }

            /* Remove gyro bias. */
            imu_gyro[0] -= gyro_bias[0];
            imu_gyro[1] -= gyro_bias[1];
            imu_gyro[2] -= gyro_bias[2];
        }

        if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, imu_accel);
        }

        if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            temperature_read_over(temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &imu_temp);
        }

        /* Map BMI088 gyro axes to the project coordinate frame. */
        float gx_deg = imu_gyro[0] * Rad2Ang;
        float gy_deg = -imu_gyro[1] * Rad2Ang; // 缈昏浆 Y 杞达紝瀵归綈 Accel
        float gz_deg = -imu_gyro[2] * Rad2Ang; // 缈昏浆 Z 杞达紝瀵归綈 Accel

        MahonyAHRSupdateIMU(gx_deg, gy_deg, gz_deg,
                            imu_accel[0], imu_accel[1], imu_accel[2]);

        /* 瑙掑害杩囬浂鐐瑰鐞嗭紙澶氬湀杩炵画瑙掑害绱锛屾秷闄ょ獊鍙橈級 */
        static float last_Yaw_a = 0.0f, last_Pitch_a = 0.0f, last_Roll_a = 0.0f;
        static float total_Yaw_a = 0.0f, total_Pitch_a = 0.0f, total_Roll_a = 0.0f;
        static uint8_t first_angle = 1;

        if (first_angle) {
            last_Yaw_a = Yaw_a; last_Pitch_a = Pitch_a; last_Roll_a = Roll_a;
            total_Yaw_a = Yaw_a; total_Pitch_a = Pitch_a; total_Roll_a = Roll_a;
            first_angle = 0;
        }

        /* 璁＄畻宸€硷紝娑堥櫎 +-180 澶勭殑璺冲彉鐜拌薄 */
        float dyaw = Yaw_a - last_Yaw_a;
        if (dyaw > 180.0f) dyaw -= 360.0f;
        else if (dyaw < -180.0f) dyaw += 360.0f;
        total_Yaw_a += dyaw;
        last_Yaw_a = Yaw_a;

        float dpitch = Pitch_a - last_Pitch_a;
        if (dpitch > 180.0f) dpitch -= 360.0f;
        else if (dpitch < -180.0f) dpitch += 360.0f;
        total_Pitch_a += dpitch;
        last_Pitch_a = Pitch_a;

        float droll = Roll_a - last_Roll_a;
        if (droll > 180.0f) droll -= 360.0f;
        else if (droll < -180.0f) droll += 360.0f;
        total_Roll_a += droll;
        last_Roll_a = Roll_a;

        /* ==================== 澶氭€佸紑鏈洪€昏緫涓庢寜閿娴?==================== */
        static uint8_t key_stable_state = 1;
        static uint8_t key_last_raw = 1;
        static uint8_t key_pressing = 0;
        static uint8_t key_long_handled = 0;
        static uint8_t key_stop_handled = 0;
        static uint8_t key_wait_second_click = 0;
        static uint32_t key_raw_change_tick = 0;
        static uint32_t key_press_tick = 0;
        static uint32_t key_first_click_tick = 0;
        static uint32_t key_hold_ms = 0;
        uint8_t key_raw_state = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET) ? 0U : 1U;
        uint32_t now_tick = (uint32_t)xTaskGetTickCount();

        if (key_raw_state != key_last_raw) {
            key_last_raw = key_raw_state;
            key_raw_change_tick = now_tick;
        }

        if ((now_tick - key_raw_change_tick) >= KEY_DEBOUNCE_MS &&
            key_stable_state != key_last_raw) {
            key_stable_state = key_last_raw;

            if (key_stable_state == 0U) {
                key_hold_ms = 0U;
                if (gimbal_state == GIMBAL_STATE_IDLE ||
                    gimbal_state == GIMBAL_STATE_RELAX) {
                    key_pressing = 1U;
                    key_long_handled = 0U;
                    key_stop_handled = 0U;
                    key_press_tick = now_tick;
                } else if (gimbal_state == GIMBAL_STATE_IMU_STAB ||
                           gimbal_state == GIMBAL_STATE_VISION_STAB) {
                    key_wait_second_click = 1U;
                    key_first_click_tick = now_tick;
                    key_pressing = 0U;
                    key_long_handled = 1U;
                    key_stop_handled = 0U;
                } else {
                    gimbal_state = GIMBAL_STATE_RELAX;
                    emergency_stop = 1;
                    gimbal_stab_target_reset = 1U;
                    key_wait_second_click = 0U;
                    key_pressing = 0U;
                    key_long_handled = 1U;
                    key_stop_handled = 1U;
                }
            } else if (key_pressing) {
                if (key_long_handled == 0U) {
                    if (gimbal_state == GIMBAL_STATE_IDLE ||
                        gimbal_state == GIMBAL_STATE_RELAX) {
                        Laser_SetOn(&g_laser, 1U);
                        gimbal_state = GIMBAL_STATE_ENC_LOCK;
                        emergency_stop = 0;
                        gimbal_stab_target_reset = 1U;
                        init_counter = 0;
                        gyro_sum[0] = 0.0f;
                        gyro_sum[1] = 0.0f;
                        gyro_sum[2] = 0.0f;
                        key_wait_second_click = 0U;
                    }
                }
                key_pressing = 0U;
                key_long_handled = 0U;
                key_stop_handled = 0U;
                key_hold_ms = 0U;
                key_press_tick = 0U;
            } else if (key_stop_handled) {
                key_long_handled = 0U;
                key_stop_handled = 0U;
                key_hold_ms = 0U;
                key_press_tick = 0U;
            }
        }

        if (key_wait_second_click != 0U &&
            (now_tick - key_first_click_tick) > KEY_DOUBLE_CLICK_MS) {
            if (gimbal_state == GIMBAL_STATE_IMU_STAB ||
                gimbal_state == GIMBAL_STATE_VISION_STAB) {
                gimbal_state = GIMBAL_STATE_RELAX;
                emergency_stop = 1;
                gimbal_stab_target_reset = 1U;
            }
            key_wait_second_click = 0U;
            key_stop_handled = 0U;
            key_long_handled = 0U;
            key_hold_ms = 0U;
            key_press_tick = 0U;
        }

        if (key_stable_state == 0U && key_pressing) {
            key_hold_ms = now_tick - key_press_tick;
            if (key_long_handled == 0U && key_hold_ms >= KEY_LONG_PRESS_MS) {
                gimbal_state = GIMBAL_STATE_GRAVITY_CALIB;
                emergency_stop = 0;
                gimbal_stab_target_reset = 1U;
                key_wait_second_click = 0U;
                Laser_SetOn(&g_laser, 1U);
                key_long_handled = 1U;
            }
        }

        /* 寮€鏈?2s 闆跺亸閲囬泦閫昏緫 (浠呭湪纭攣鎬佷笅杩涜锛屾鏃舵満姊扮粷瀵归潤姝? */
        if (gimbal_state == GIMBAL_STATE_CALIBRATING)
        {
            init_counter++;
            if (init_counter >= STAB_CALIB_SAMPLE_COUNT)
            {
                // 璁＄畻鍑鸿繖 2000 娆￠噰鏍风殑骞冲潎瑙掗€熷害锛屼綔涓烘柊鐨勭粷瀵归浂鍋忥紒
                gyro_bias[0] = gyro_sum[0] / (float)STAB_CALIB_SAMPLE_COUNT;
                gyro_bias[1] = gyro_sum[1] / (float)STAB_CALIB_SAMPLE_COUNT;
                gyro_bias[2] = gyro_sum[2] / (float)STAB_CALIB_SAMPLE_COUNT;
                
                // 灏嗗綋鍓嶇殑绱Н瑙掑害璁句负闆剁偣
                yaw_offset = total_Yaw_a;
                pitch_offset = total_Pitch_a;
                roll_offset = total_Roll_a;
                
                gimbal_state = GIMBAL_STATE_VISION_STAB;
            }
        }
        
        float out_yaw = fmodf(total_Yaw_a - yaw_offset, 360.0f);
        if (out_yaw < 0.0f) out_yaw += 360.0f;  // 纭繚濮嬬粓鍦?[0, 360)
        float out_pitch = total_Pitch_a - pitch_offset;
        float out_roll = total_Roll_a - roll_offset;

        /* ======================== 鍏ㄥ眬濮挎€佹洿鏂颁笌钃濈墮杈撳嚭 ======================== */
        // 缂栫爜鍣ㄨ搴?(CAN灞傚凡杞负瑙掑害鍒?0~360掳)
        float enc_pitch_deg = pitch_control_angle(pitch_motor.angle);
        
        // 浜戝彴鏈€缁?Pitch 鍜?Yaw (IMU + 缂栫爜鍣ㄨ瀺鍚?
        float gimbal_pitch = out_pitch + (enc_pitch_deg - MOTOR_PITCH_MECH_ZERO);
        float gimbal_yaw   = out_yaw;
        
        global_gimbal_pitch = gimbal_pitch;
        global_gimbal_yaw   = gimbal_yaw;
        global_imu_yaw_speed   = gz_deg; 
        global_imu_pitch_speed = gy_deg;

        vision_raw_dx = Vision_GetDx(&g_vision);
        vision_raw_dy = Vision_GetDy(&g_vision);
        vision_raw_valid = Vision_IsValid(&g_vision);
        vision_raw_frame_count = Vision_GetFrameCount(&g_vision);
        vision_raw_frame_seq = Vision_GetFrameSeq(&g_vision);

        if ((now_tick - vision_last_tx_tick) >= 50U)
        {
            vision_last_tx_tick = now_tick;

            int16_t  vis_dx  = Vision_GetDx(&g_vision);
            int16_t  vis_dy  = Vision_GetDy(&g_vision);
            uint32_t vis_frame_count = Vision_GetFrameCount(&g_vision);
            uint32_t vis_frame_seq = Vision_GetFrameSeq(&g_vision);

            uint32_t now_ms = (uint32_t)xTaskGetTickCount();
            static char vision_tx_buf[180];
            sprintf(vision_tx_buf, "M,STAT,%lu,%u,%d,%d,%u,%lu,%lu,%.2f,%.2f,%.2f,%.2f,%u,%.3f,%.3f\r\n",
                    now_ms,
                    (unsigned)gimbal_state,
                    vis_dx,
                    vis_dy,
                    (unsigned)Vision_IsValid(&g_vision),
                    vis_frame_count,
                    vis_frame_seq,
                    gimbal_pitch,
                    gimbal_yaw,
                    pitch_motor.current,
                    yaw_motor.current,
                    (unsigned)MODE_VISION_STAB_GetVisionActive(),
                    MODE_VISION_STAB_GetVisionPitchOffset(),
                    MODE_VISION_STAB_GetVisionYawOffset());
            Vision_Send(&g_vision, (const uint8_t *)vision_tx_buf, strlen(vision_tx_buf));
        }

        /* 鏋佺淇濇姢锛氬鏋滅敱浜庣灛闂村苟鍙戞暟鎹鐖嗕簡 RX_FIFO 瀵艰嚧 CAN 纭欢鎶ュ嚭 Overrun 閿欒閿佹锛?           鎴戜滑蹇呴』寮鸿娓呴櫎閿欒骞堕噸鏂版墦寮€涓柇锛屽惁鍒欏悗缁案杩滆繘涓嶄簡鎺ユ敹鍥炶皟锛?*/
        extern CAN_HandleTypeDef hcan1;
        if (hcan1.ErrorCode != HAL_CAN_ERROR_NONE) {
            hcan1.ErrorCode = HAL_CAN_ERROR_NONE;
            __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
            __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV1);
            HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
    }
}

/* ======================== 钃濈墮鎺ユ敹浠诲姟 ======================== */

void BT_RxTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /* 钃濈墮鎺ユ敹鍦?ISR 涓畬鎴?(IDLE + DMA)
         * 姝や换鍔￠鐣欑敤浜庢墿灞?*/
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ======================== DMA 鏁版嵁瑙ｆ瀽 (鍙傝€?QGimbal) ======================== */

static void gyro_read_over(const uint8_t *rx_buf, float gyro_out[3])
{
    int16_t raw;
    raw = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    gyro_out[0] = raw * BMI088_GYRO_2000_SEN;
    raw = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    gyro_out[1] = raw * BMI088_GYRO_2000_SEN;
    raw = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    gyro_out[2] = raw * BMI088_GYRO_2000_SEN;
}

static void accel_read_over(const uint8_t *rx_buf, float accel_out[3])
{
    int16_t raw;
    raw = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    accel_out[0] = raw * BMI088_ACCEL_3G_SEN;
    raw = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    accel_out[1] = raw * BMI088_ACCEL_3G_SEN;
    raw = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    accel_out[2] = raw * BMI088_ACCEL_3G_SEN;
}

static void temperature_read_over(const uint8_t *rx_buf, float *temp_out)
{
    int16_t raw = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));
    if (raw > 1023) raw -= 2048;
    *temp_out = raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

/* ======================== SPI DMA 璋冨害 (鍙傝€?QGimbal) ======================== */

/**
 * @brief  鏍规嵁鏍囧織浣嶅惎鍔ㄤ笅涓€娆?SPI DMA 浼犺緭
 *         浼樺厛绾? gyro > accel > temp
 */
static void imu_cmd_spi_dma(void)
{
    /* 闄€铻轰华 DMA */
    if ((gyro_update_flag & (1 << IMU_DR_SHFITS))
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS))
        && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        BSP_SPI_GyroCS_Low();
        BSP_SPI_DMA_Enable((uint32_t)gyro_dma_tx_buf,
                           (uint32_t)gyro_dma_rx_buf,
                           SPI_DMA_GYRO_LEN);
        return;
    }

    /* 鍔犻€熷害璁?DMA */
    if ((accel_update_flag & (1 << IMU_DR_SHFITS))
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        BSP_SPI_AccelCS_Low();
        BSP_SPI_DMA_Enable((uint32_t)accel_dma_tx_buf,
                           (uint32_t)accel_dma_rx_buf,
                           SPI_DMA_ACCEL_LEN);
        return;
    }

    /* 娓╁害 DMA */
    if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS))
        && !(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        BSP_SPI_AccelCS_Low();
        BSP_SPI_DMA_Enable((uint32_t)temp_dma_tx_buf,
                           (uint32_t)temp_dma_rx_buf,
                           SPI_DMA_TEMP_LEN);
        return;
    }
}

/* ======================== EXTI DRDY 涓柇鍥炶皟 ======================== */

/**
 * @brief  GPIO EXTI 鍥炶皟 (HAL 妗嗘灦璋冪敤)
 *         PC4 = INT1_ACCEL (DRDY), PC5 = INT1_GYRO (DRDY)
 *         鍙傝€?QGimbal: HAL_GPIO_EXTI_Callback
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= (1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_DR_SHFITS);
        if (imu_start_dma_flag)
            imu_cmd_spi_dma();
    }
    else if (GPIO_Pin == INT1_GRYO_Pin)
    {
        gyro_update_flag |= (1 << IMU_DR_SHFITS);
        if (imu_start_dma_flag)
            imu_cmd_spi_dma();
    }
}

/* ======================== SPI DMA 瀹屾垚鍥炶皟 ======================== */

/**
 * @brief  SPI1 RX DMA 瀹屾垚鍥炶皟 (瑕嗙洊 BSP 寮卞畾涔?
 *         鍙傝€?QGimbal: DMA2_Stream0_IRQHandler 鍐呴儴閫昏緫
 */
void BSP_SPI_DMA_RxCpltCallback(void)
{
    /* 闄€铻轰华璇诲彇瀹屾垚 */
    if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
    {
        gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
        gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);
        BSP_SPI_GyroCS_High();
    }

    /* 鍔犻€熷害璁¤鍙栧畬鎴?*/
    if (accel_update_flag & (1 << IMU_SPI_SHFITS))
    {
        accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
        accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
        BSP_SPI_AccelCS_High();
    }

    /* 娓╁害璇诲彇瀹屾垚 */
    if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
    {
        accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
        accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);
        BSP_SPI_AccelCS_High();
    }

    /* 缁х画涓嬩竴涓?DMA 浼犺緭 */
    imu_cmd_spi_dma();

    /* 闄€铻轰华鏁版嵁灏辩华鏃堕€氱煡 INS 浠诲姟 */
    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        if (ins_task_handle_local != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(ins_task_handle_local, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/* ======================== 鏁版嵁鑾峰彇 ======================== */

float INS_GetYaw(void)       { return Yaw_a; }
float INS_GetPitch(void)     { return Pitch_a; }
float INS_GetRoll(void)      { return Roll_a; }
float INS_GetYawTotal(void)  { return Yaw_TotalAngle; }
uint8_t INS_IsCalibrated(void) { return g_bmi088.calibrated; }
