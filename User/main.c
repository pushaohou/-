/**
 * @file    main.c
 * @brief   智能导盲杖 — 主调度器
 *
 *          每个模块自包含，main.c 只负责:
 *          1. 初始化所有模块
 *          2. 按节拍调用各模块的 Update
 *          3. 综合各模块状态 → 驱动蜂鸣器 / OLED / 蓝牙
 *
 *          调度节拍:
 *          - 每 100ms: 超声波测距 (DIST)
 *          - 每  50ms: 姿态 + 跌倒检测 (SAFE)
 *          - 每 200ms: OLED 刷新
 *          - 每 500ms: 蓝牙数据上报
 *          - 实时:     蜂鸣器模式更新
 */
#include "debug.h"
#include "../Hardware/oled.h"
#include "../Hardware/buzzer.h"
#include "../Hardware/bluetooth.h"
#include "../App/dist.h"
#include "../App/safe.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

    printf("\r\n");
    printf("====================================\r\n");
    printf("  Smart Guide Cane - CH32V307VCT   \r\n");
    printf("  SystemClk: %d Hz                 \r\n", SystemCoreClock);
    printf("  ChipID: %08x                    \r\n", DBGMCU_GetCHIPID());
    printf("====================================\r\n");
    printf("=== Guide Cane Starting ===\r\n");

    /*---- 模块初始化 ----*/
    printf("Init OLED...\r\n");
    OLED_Init();
    OLED_ShowString(0, 0, "Guide Cane", OLED_FONT_6x8);
    OLED_ShowString(0, 2, "Starting...", OLED_FONT_6x8);

    printf("Init HC-SR04...\r\n");
    DIST_Init();

    printf("Init Buzzer...\r\n");
    Buzzer_Init();

    printf("Init Bluetooth...\r\n");
    BT_Init();

    SAFE_Init();  /* 放最后 — 内部有 printf 输出 MPU6050 状态 */

    /* 屏幕显示结果 — 先复位 I2C1 确保通信正常 */
    I2C_Cmd(OLED_I2C, DISABLE);
    I2C_Cmd(OLED_I2C, ENABLE);
    OLED_Clear();
    OLED_ShowString(0, 0, "Guide Cane", OLED_FONT_6x8);
    OLED_ShowString(0, 2, "Status:READY", OLED_FONT_6x8);

    printf("=== Guide Cane Ready ===\r\n");
    BT_SendString("Guide Cane Online\r\n");

    /* 蜂鸣器自检 — 短鸣一声 */
    Buzzer_On();
    Delay_Ms(200);
    Buzzer_Off();

    /*---- 局部定时变量（不再使用全局上下文结构体）----*/
    uint32_t sys_ms          = 0;
    uint32_t last_measure_ms = 0;
    uint32_t last_oled_ms    = 0;
    uint32_t last_bt_ms      = 0;

    while (1)
    {
        sys_ms++;

        /*---- 每 100ms 超声波测距 ----*/
        if (sys_ms - last_measure_ms >= 100) {
            last_measure_ms = sys_ms;
            DIST_Update();
        }

        /*---- 每 50ms MPU6050 姿态 + 跌倒检测 ----*/
        if (sys_ms % 50 == 0) {
            SAFE_Update();
        }

        /*---- 综合状态判断 ----*/
        SystemState state;
        if (SAFE_IsFall()) {
            state = STATE_FALL;
        } else if (DIST_GetFrontLevel() == OBSTACLE_NEAR ||
                   DIST_GetLeftLevel()  == OBSTACLE_NEAR ||
                   DIST_GetRightLevel() == OBSTACLE_NEAR) {
            state = STATE_DANGER;
        } else if (DIST_GetFrontLevel() == OBSTACLE_FAR ||
                   DIST_GetLeftLevel()  == OBSTACLE_FAR ||
                   DIST_GetRightLevel() == OBSTACLE_FAR) {
            state = STATE_WARNING;
        } else {
            state = STATE_NORMAL;
        }

        /*---- 蜂鸣器反馈（方向感知，一回状态变化触发一次）----*/
        {
            Buzzer_Pattern buzz = BUZZER_OFF;

            if (state == STATE_FALL) {
                buzz = BUZZER_FALL;
            } else if (state == STATE_DANGER || state == STATE_WARNING) {
                uint16_t df = DIST_GetFront();
                uint16_t dl = DIST_GetLeft();
                uint16_t dr = DIST_GetRight();

                /* 找到最近障碍物方向（0=无，视为无穷远） */
                if (df == 0) df = 9999;
                if (dl == 0) dl = 9999;
                if (dr == 0) dr = 9999;

                if (df <= dl && df <= dr)
                    buzz = BUZZER_FRONT;
                else if (dl <= df && dl <= dr)
                    buzz = BUZZER_LEFT;
                else
                    buzz = BUZZER_RIGHT;
            }

            Buzzer_Trigger(buzz);
        }
        Buzzer_Process();

        /*---- 每 200ms OLED 刷新 ----*/
        if (sys_ms - last_oled_ms >= 200) {
            last_oled_ms = sys_ms;

            char line[22];

            /* 第一行：距离 */
            snprintf(line, sizeof(line), "F:%03u L:%03u R:%03u",
                     DIST_GetFront(), DIST_GetLeft(), DIST_GetRight());
            OLED_ShowString(0, 0, line, OLED_FONT_6x8);

            /* 第二行：系统状态 */
            switch (state) {
                case STATE_NORMAL:  snprintf(line, sizeof(line), "Status:OK        "); break;
                case STATE_WARNING: snprintf(line, sizeof(line), "Status:WARN !!!  "); break;
                case STATE_DANGER:  snprintf(line, sizeof(line), "Status:DANGER!!! "); break;
                case STATE_FALL:    snprintf(line, sizeof(line), "Status:FALL!!!   "); break;
            }
            OLED_ShowString(0, 2, line, OLED_FONT_6x8);

            /* 第三行：倾角（×10整数，避免 nano 库不支持 %f） */
            {
                int p = (int)(SAFE_GetPitch() * 10);
                int r = (int)(SAFE_GetRoll()  * 10);
                char ps = (p < 0) ? '-' : '+'; if (p < 0) p = -p;
                char rs = (r < 0) ? '-' : '+'; if (r < 0) r = -r;
                snprintf(line, sizeof(line), "P:%c%d.%d R:%c%d.%d",
                         ps, p / 10, p % 10, rs, r / 10, r % 10);
            }
            OLED_ShowString(0, 4, line, OLED_FONT_6x8);

            /* 第四行：蓝牙 + 蜂鸣器状态（方向） */
            {
                const char *buzz_str = "OFF";
                switch (state) {
                    case STATE_WARNING: buzz_str = "WARN";  break;
                    case STATE_DANGER:  buzz_str = "DANG";  break;
                    case STATE_FALL:    buzz_str = "FALL";  break;
                    default:            buzz_str = "OFF";   break;
                }
                snprintf(line, sizeof(line), "BT:ON  BZ:%s", buzz_str);
                OLED_ShowString(0, 6, line, OLED_FONT_6x8);
            }
        }

        /*---- 每 500ms 蓝牙上报 ----*/
        if (sys_ms - last_bt_ms >= 500) {
            last_bt_ms = sys_ms;

            BT_AlertLevel bt_alert;
            switch (state) {
                case STATE_WARNING: bt_alert = ALERT_WARNING; break;
                case STATE_DANGER:  bt_alert = ALERT_DANGER;  break;
                case STATE_FALL:    bt_alert = ALERT_FALL;    break;
                default:            bt_alert = ALERT_NONE;    break;
            }

            BT_SendData(DIST_GetFront(), DIST_GetLeft(), DIST_GetRight(),
                        SAFE_GetPitch(), SAFE_GetRoll(), bt_alert);
        }

        /* 延时 1ms（主循环节拍）*/
        Delay_Ms(1);
    }
}
