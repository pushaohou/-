/**
 * @file    safe.c
 * @brief   安全/跌倒检测模块 — 实物验证版本
 * @note    跌倒检测逻辑与参考项目完全一致:
 *          - 只检测倾角 > 60°（不检测加速度冲击）
 *          - 连续 5 次确认后才判定跌倒（~160ms 抗干扰）
 *          - 倾角回落自动清除跌倒标志（自动恢复）
 *          - 调试输出: $SAFE,fall_flag,angle#
 */
#include "safe.h"
#include "debug.h"

/* 跌倒确认次数（参考项目实测值，约 160ms 去抖） */
#define FALL_CONFIRM_COUNT  5

/* 全局状态 */
static uint8_t  g_fall_flag;
static int16_t  g_angle;
static uint8_t  g_fall_count;

/* 原始数据和姿态（供 OLED/BT 显示用） */
static MPU6050_Data     g_raw;
static MPU6050_Attitude g_att;
static SystemState      g_state;

/* ==================== 整数开平方（与参考一致） ==================== */
static uint32_t isqrt(uint32_t n)
{
    if (n <= 1) return n;
    uint32_t x = n;
    uint32_t y = (x + 1) >> 1;
    while (y < x) {
        x = y;
        y = (x + n / x) >> 1;
    }
    return x;
}

/* ==================== 倾角计算（与参考一致的 acos 查表法） ==================== */
static int16_t calc_tilt_angle(int32_t az, uint32_t mag)
{
    if (mag == 0) return 0;

    int32_t c = (az * 1000) / (int32_t)mag;
    if (c > 1000)  c = 1000;
    if (c < -1000) c = -1000;

    /* 21 个刻度的 arccos 表 ×10，覆盖 0°~90° */
    static const int16_t acos_tbl[] = {
          0, 258, 369, 456, 531, 600, 664, 725, 784, 843,
        900, 956,1016,1075,1136,1200,1269,1344,1428,1527,1800
    };

    int32_t idx;
    if (c >= 0) {
        idx = (1000 - c) / 100;
    } else {
        idx = 10 + (-c) / 100;
    }
    if (idx > 20) idx = 20;

    return acos_tbl[idx] / 10;
}

/* ==================== SAFE_Init ==================== */
void SAFE_Init(void)
{
    uint8_t mpu_ok;

    printf("Init MPU6050...\r\n");
    mpu_ok = MPU6050_Init();
    if (mpu_ok) {
        printf("MPU6050 OK\r\n");
    } else {
        printf("MPU6050 FAIL!\r\n");
    }

    g_fall_flag  = 0;
    g_angle      = 0;
    g_fall_count = 0;
    g_state      = STATE_NORMAL;
}

/* ==================== SAFE_Update（与参考一致） ==================== */
void SAFE_Update(void)
{
    /* 读取 MPU6050 原始加速度 */
    MPU6050_ReadAll(&g_raw);

    int16_t ax = g_raw.accel_x;
    int16_t ay = g_raw.accel_y;
    int16_t az = g_raw.accel_z;

    /* 计算加速度合成值（整数运算） */
    int32_t mag_sq = (int32_t)ax * ax + (int32_t)ay * ay + (int32_t)az * az;
    uint32_t mag = isqrt((uint32_t)mag_sq);

    /* 倾角计算（与参考一致：acos 查表法） */
    g_angle = calc_tilt_angle(az, mag);

    /* 跌倒检测：倾角 > 60°，连续 FALL_CONFIRM_COUNT 次确认 */
    if (g_angle > FALL_TILT_ANGLE)
    {
        g_fall_count++;
        if (g_fall_count >= FALL_CONFIRM_COUNT)
        {
            g_fall_flag  = 1;
            g_fall_count = FALL_CONFIRM_COUNT;
            g_state      = STATE_FALL;
        }
    }
    else
    {
        g_fall_count = 0;
        g_fall_flag  = 0;
        g_state      = STATE_NORMAL;
    }

    /* 同时计算浮点姿态角（OLED 显示用） */
    MPU6050_CalcAttitude(&g_raw, &g_att);

    /* 调试输出（与参考一致格式） */
    printf("$SAFE,%d,%d#\r\n", g_fall_flag, g_angle);
}

/* ==================== 对外接口 ==================== */
uint8_t     SAFE_IsFall(void)   { return g_fall_flag; }
int16_t     SAFE_GetAngle(void) { return g_angle; }
float       SAFE_GetPitch(void) { return g_att.pitch; }
float       SAFE_GetRoll(void)  { return g_att.roll; }
SystemState SAFE_GetState(void) { return g_state; }
