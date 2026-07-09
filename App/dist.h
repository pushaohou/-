/**
 * @file    dist.h
 * @brief   超声波距离模块 — 包装 HC-SR04 驱动，拥有障碍物分级逻辑
 * @note    参考 SAFE/DIST 模块化设计，Init / Update / Get 三部曲
 */
#ifndef __DIST_H
#define __DIST_H

#include "../Hardware/guide_cane_config.h"
#include "../Hardware/hcsr04.h"

/* 障碍物等级 */
typedef enum {
    OBSTACLE_CLEAR = 0,
    OBSTACLE_FAR   = 1,
    OBSTACLE_NEAR  = 2
} ObstacleLevel;

void          DIST_Init(void);
void          DIST_Update(void);

uint16_t      DIST_GetFront(void);
uint16_t      DIST_GetLeft(void);
uint16_t      DIST_GetRight(void);

ObstacleLevel DIST_GetFrontLevel(void);
ObstacleLevel DIST_GetLeftLevel(void);
ObstacleLevel DIST_GetRightLevel(void);

#endif /* __DIST_H */
