/**
 * @file    dist.c
 * @brief   超声波距离模块 — 三路 HC-SR04 测距 + 障碍物等级判定
 * @note    内部持有测距结果和等级缓存，Init / Update / Get 模式
 */
#include "dist.h"

static HCSR04_Result g_result;
static ObstacleLevel g_front_level;
static ObstacleLevel g_left_level;
static ObstacleLevel g_right_level;

/* 距离 → 障碍物等级 */
static ObstacleLevel DistanceToLevel(uint16_t cm)
{
    if (cm == 0)              return OBSTACLE_CLEAR;
    if (cm <= OBSTACLE_NEAR_CM) return OBSTACLE_NEAR;
    if (cm <= OBSTACLE_MID_CM)  return OBSTACLE_FAR;
    return OBSTACLE_CLEAR;
}

void DIST_Init(void)
{
    HCSR04_Init();
    g_result.front_cm = 0;
    g_result.left_cm  = 0;
    g_result.right_cm = 0;
    g_front_level = OBSTACLE_CLEAR;
    g_left_level  = OBSTACLE_CLEAR;
    g_right_level = OBSTACLE_CLEAR;
}

void DIST_Update(void)
{
    HCSR04_MeasureAll(&g_result);

    g_front_level = DistanceToLevel(g_result.front_cm);
    g_left_level  = DistanceToLevel(g_result.left_cm);
    g_right_level = DistanceToLevel(g_result.right_cm);
}

uint16_t DIST_GetFront(void) { return g_result.front_cm; }
uint16_t DIST_GetLeft(void)  { return g_result.left_cm; }
uint16_t DIST_GetRight(void) { return g_result.right_cm; }

ObstacleLevel DIST_GetFrontLevel(void) { return g_front_level; }
ObstacleLevel DIST_GetLeftLevel(void)  { return g_left_level; }
ObstacleLevel DIST_GetRightLevel(void) { return g_right_level; }
