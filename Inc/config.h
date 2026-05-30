#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * ========================================================
 * 强动力 (QD) 电机参数配置 (Motor Parameter Config)
 * 根据您提供的图片参数：
 * ========================================================
 */

#define MOTOR_RATED_VOLTAGE_MIN  7     // 额定电压最小值 (V)
#define MOTOR_RATED_VOLTAGE_MAX  26    // 额定电压最大值 (V)
#define MOTOR_RATED_CURRENT      1.0f  // 额定电流 (A)
#define MOTOR_PEAK_CURRENT       2.0f  // 峰值电流 (A)
#define MOTOR_RATED_TORQUE       0.2f  // 额定扭矩 (Nm)
#define MOTOR_PEAK_TORQUE        0.3f  // 峰值扭矩 (Nm)
#define MOTOR_RATED_SPEED        500   // 额定转速 (rpm)
#define MOTOR_PEAK_SPEED         800   // 峰值转速 (rpm)
#define MOTOR_POLE_PAIRS         14    // 极对数
#define MOTOR_ENCODER_BITS       15    // 编码器位数 (bit)
#define MOTOR_CONTROL_ACCURACY   0.0109f // 控制精度 (°)

/*
 * 通信方式: CAN, USB, UART
 * CAN 波特率: 1 Mbps
 * UART 波特率: 115200 bps
 * 控制模式: 电流模式、速度模式、角度模式、低速大力矩模式、角度步进模式
 */

/*
 * --------------------------------------------------------
 * 电机内部控制环参数与限制 (导出的 config)
 * --------------------------------------------------------
 */
#define MOTOR_PID_SPEED_KP       0.00300f
#define MOTOR_PID_SPEED_KI       0.000390f
#define MOTOR_PID_SPEED_KD       0.000f

#define MOTOR_PID_ANGLE_KP       1200.0f
#define MOTOR_PID_ANGLE_KI       0.000f
#define MOTOR_PID_ANGLE_KD       0.000f

#define MOTOR_LIMIT_SPEED        1000.0f  // 速度限制 (rpm)
#define MOTOR_LIMIT_CURRENT      1.65f    // 电流限制 (A)

/*
 * --------------------------------------------------------
 * 机械结构参数 (校准后填入)
 * --------------------------------------------------------
 */
#define MOTOR_PITCH_MECH_ZERO    0.0f     // Pitch 轴机械零点 (单位: 度)
#define MOTOR_YAW_MECH_ZERO      0.0f     // Yaw 轴机械零点 (单位: 度)

/*
 * --------------------------------------------------------
 * 云台外环 (1000Hz) PID 增稳控制参数 (极度保守)
 * --------------------------------------------------------
 */
// 极性宏：极其重要！如果发生正反馈震荡（一碰就加速甩头），请将其改为 -1.0f
#define PITCH_POLARITY 1.0f
#define YAW_POLARITY   -1.0f

/* IMU attitude loop PID. Pitch starts with conservative values for safety. */
#define GIMBAL_PID_PITCH_KP 0.030f
#define GIMBAL_PID_PITCH_KI 0.0f
#define GIMBAL_PID_PITCH_KD 0.0055f
#define GIMBAL_PID_YAW_KP   0.050f
#define GIMBAL_PID_YAW_KI   0.0002f
#define GIMBAL_PID_YAW_KD   0.0010f

/* Stab mode output switches. */
#define GIMBAL_STAB_PITCH_ENABLE 1
#define GIMBAL_STAB_YAW_ENABLE   1

/* Pitch attitude loop safety gates. Keep integral disabled until P/D are stable. */
#define GIMBAL_STAB_PITCH_MAX_CURRENT         0.75f
#define GIMBAL_STAB_PITCH_MAX_ERR_DEG         16.0f
#define GIMBAL_STAB_PITCH_DEADBAND_DEG        0.3f
#define GIMBAL_STAB_PITCH_ATTITUDE_P_ENABLE   1
#define GIMBAL_STAB_PITCH_I_ENABLE            0
#define GIMBAL_STAB_PITCH_I_SEPARATE_ERR_DEG  6.0f
#define GIMBAL_STAB_PITCH_I_MAX_SPEED_DPS     10.0f

/* Encoder angle loops. These are independent from IMU stab PID.
 * Output current = (Kp * angle_err + I - Kd * motor_speed) * polarity.
 */
#define ANGLE_LOOP_FREQ_HZ                   1000U
#define ANGLE_LOOP_DT_SEC                    0.001f

#define PITCH_ANGLE_LOOP_KP                  0.038f
#define PITCH_ANGLE_LOOP_KI                  0.006f
#define PITCH_ANGLE_LOOP_KD                  0.0045f
#define PITCH_ANGLE_LOOP_POLARITY            1.0f
#define PITCH_ANGLE_LOOP_DEADBAND_DEG        0.3f
#define PITCH_ANGLE_LOOP_MAX_ERR_DEG         14.0f
#define PITCH_ANGLE_LOOP_I_SEPARATE_ERR_DEG  1.5f
#define PITCH_ANGLE_LOOP_MAX_I_CURRENT       0.025f
#define PITCH_ANGLE_LOOP_MAX_CURRENT         0.75f
#define PITCH_ANGLE_LOOP_FF_ERR_DEG          0.8f
#define PITCH_ANGLE_LOOP_POS_FF_CURRENT      0.0f
#define PITCH_ANGLE_LOOP_NEG_FF_CURRENT      0.0f

#define YAW_ANGLE_LOOP_KP                    0.050f
#define YAW_ANGLE_LOOP_KI                    0.020f
#define YAW_ANGLE_LOOP_KD                    0.0035f
#define YAW_ANGLE_LOOP_POLARITY              1.0f
#define YAW_ANGLE_LOOP_DEADBAND_DEG          0.25f
#define YAW_ANGLE_LOOP_MAX_ERR_DEG           10.0f
#define YAW_ANGLE_LOOP_I_SEPARATE_ERR_DEG    3.0f
#define YAW_ANGLE_LOOP_MAX_I_CURRENT         0.04f
#define YAW_ANGLE_LOOP_MAX_CURRENT           0.45f
#define YAW_ANGLE_LOOP_FF_ERR_DEG            0.45f
#define YAW_ANGLE_LOOP_POS_FF_CURRENT        0.035f
#define YAW_ANGLE_LOOP_NEG_FF_CURRENT        0.070f

/* Pitch/Yaw angle-loop automatic test. */
#define ENABLE_ANGLE_TEST_MODE                0
#define ANGLE_TEST_STEP_DEG                  5.0f
#define ANGLE_TEST_PHASE_MS                  1500U
#define ANGLE_TEST_LIMIT_MARGIN_DEG          5.0f

/* Open-loop torque/friction threshold test. */
#define ENABLE_TORQUE_TEST_MODE              0
#define TORQUE_TEST_RAMP_STEP_CURRENT        0.002f
#define TORQUE_TEST_MAX_CURRENT              0.35f
#define TORQUE_TEST_MOVE_DEG                 0.35f
#define TORQUE_TEST_SETTLE_MS                300U

// 控制死区 (度)，在此误差范围内视为已到达目标，不进行 P 环修正，消除微抖动
#define GIMBAL_PITCH_DEADBAND 0.5f
#define GIMBAL_YAW_DEADBAND   0.2f

// === 核心保护：误差限幅 (极度重要，防画龙防疯转) ===
// 无论外面怎么扰动，系统认为的“最大误差”不超过此值。防止放手瞬间积累极大动能。
#define GIMBAL_MAX_ERR_PITCH  20.0f
#define GIMBAL_MAX_ERR_YAW    20.0f

// 积分项限幅 (防积分饱和)
#define GIMBAL_MAX_I_PITCH    0.25f
#define GIMBAL_MAX_I_YAW      0.3f

// 安全限幅保护 (最大输出指令电流 A，用手能轻松捏住，防烧毁防击穿)
#define GIMBAL_MAX_OUTPUT_CURRENT 1.0f

/* Pitch encoder soft clamp. Absolute encoder angle is 0~360 deg. */
#define PITCH_SOFT_LIMIT_MIN_DEG 19.0f
#define PITCH_SOFT_LIMIT_MAX_DEG 112.0f
#define PITCH_CURRENT_POS_INCREASES_ANGLE 1.0f

/* Pitch gravity feedforward:
 * Iff = sin_k * sin(enc_deg) + cos_k * cos(enc_deg) + offset.
 * Fit these three coefficients from enc/current samples.
 */
#define PITCH_GRAVITY_FF_ENABLE 1
#define PITCH_GRAVITY_FF_SIN_K 0.0300124f
#define PITCH_GRAVITY_FF_COS_K 0.0200932f
#define PITCH_GRAVITY_FF_OFFSET 0.0038665f
#define PITCH_GRAVITY_FF_MAX_CURRENT 0.6f

/* Long press PA0 enters gravity calibration sweep. */
#define KEY_DEBOUNCE_MS 30U
#define KEY_LONG_PRESS_MS 1200U
#define KEY_DOUBLE_CLICK_MS 350U
#define STAB_CALIB_SAMPLE_COUNT 2000U
#define PITCH_GRAVITY_CALIB_POINT_COUNT      9U
#define PITCH_GRAVITY_CALIB_LIMIT_MARGIN_DEG 6.0f
#define PITCH_GRAVITY_CALIB_SETTLE_MS        1200U
#define PITCH_GRAVITY_CALIB_SAMPLE_MS        900U

/* Vision closed-loop protection. Maix sends pixel error only; MCU owns safety. */
#define VISION_LOOP_ENABLE                    1
#define VISION_LOOP_PERIOD_MS                 10U
#define VISION_INPUT_TIMEOUT_MS               250U
#define VISION_INPUT_MAX_DX                   180
#define VISION_INPUT_MAX_DY                   140
#define VISION_INPUT_DEADBAND_X               2.0f
#define VISION_INPUT_DEADBAND_Y               2.0f
#define VISION_INTEGRAL_DEADBAND_X            1.0f
#define VISION_INTEGRAL_DEADBAND_Y            1.0f
#define VISION_INTEGRAL_CENTER_DECAY          0.90f
#define VISION_INTEGRAL_REVERSAL_DECAY        0.50f
#define VISION_YAW_POLARITY                   1.0f
#define VISION_PITCH_POLARITY                 1.0f
#define VISION_YAW_PID_KP_DEG_PER_PIXEL       0.00024f
#define VISION_YAW_PID_KI_DEG_PER_PIXEL       0.000004f
#define VISION_YAW_PID_KD_DEG_PER_PIXEL       0.0f
#define VISION_YAW_PID_I_MAX_DEG              0.008f
#define VISION_PITCH_PID_KP_DEG_PER_PIXEL     0.00018f
#define VISION_PITCH_PID_KI_DEG_PER_PIXEL     0.000003f
#define VISION_PITCH_PID_KD_DEG_PER_PIXEL     0.0f
#define VISION_PITCH_PID_I_MAX_DEG            0.006f
#define VISION_YAW_TRACK_MAX_STEP_DEG         0.015f
#define VISION_PITCH_TRACK_MAX_STEP_DEG       0.012f
#define VISION_CIRCLE_YAW_PID_KP_DEG_PER_PIXEL    0.00036f
#define VISION_CIRCLE_YAW_PID_KI_DEG_PER_PIXEL    0.0000024f
#define VISION_CIRCLE_YAW_PID_KD_DEG_PER_PIXEL    0.000070f
#define VISION_CIRCLE_YAW_PID_I_MAX_DEG           0.018f
#define VISION_CIRCLE_PITCH_PID_KP_DEG_PER_PIXEL  0.00027f
#define VISION_CIRCLE_PITCH_PID_KI_DEG_PER_PIXEL  0.0000018f
#define VISION_CIRCLE_PITCH_PID_KD_DEG_PER_PIXEL  0.000052f
#define VISION_CIRCLE_PITCH_PID_I_MAX_DEG         0.014f
#define VISION_CIRCLE_YAW_TRACK_MAX_STEP_DEG      0.034f
#define VISION_CIRCLE_PITCH_TRACK_MAX_STEP_DEG    0.028f
#define VISION_CIRCLE_INPUT_FILTER_ALPHA          0.72f
#define VISION_CIRCLE_PREDICT_ENABLE              0
#define VISION_CIRCLE_PREDICT_MAX_MS              70U
#define VISION_CIRCLE_PREDICT_GAIN                0.35f
#define VISION_CIRCLE_PREDICT_MAX_PX              8.0f
#define VISION_CIRCLE_OUTPUT_MAX_SPEED_PX_S       140.0f
#define VISION_LOST_SCAN_ENABLE               1
#define VISION_LOST_SCAN_DELAY_MS             500U
#define VISION_LOST_SCAN_YAW_LIMIT_DEG        45.0f
#define VISION_LOST_SCAN_SPEED_DPS            8.0f

#endif /* __CONFIG_H__ */
