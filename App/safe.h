/**
 * @file    safe.h
 * @brief   安全/跌倒检测模块 — 包装 MPU6050 驱动，拥有姿态解算和跌倒判断
 * @note    跌倒检测逻辑与参考项目实物验证版本一致:
 *          - 倾角 > 60° 连续 5 次确认 ≈ 160ms
 *          - 倾角回落后自动清除跌倒标志
 */
#ifndef __SAFE_H
#define __SAFE_H

#include "../Hardware/guide_cane_config.h"
#include "../Hardware/mpu6050.h"
#include <stdint.h>

/* 系统状态 */
typedef enum {
    STATE_NORMAL   = 0,
    STATE_WARNING  = 1,
    STATE_DANGER   = 2,
    STATE_FALL     = 3
} SystemState;

void        SAFE_Init(void);
void        SAFE_Update(void);

uint8_t     SAFE_IsFall(void);
int16_t     SAFE_GetAngle(void);
float       SAFE_GetPitch(void);
float       SAFE_GetRoll(void);
SystemState SAFE_GetState(void);

#endif /* __SAFE_H */
