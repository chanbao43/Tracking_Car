/* ============================================================
 * Test 7: 第二阶段专项测试 — 跳过第一圈，直测 180° 掉头 + 90° 弯道巡线
 * 基于: main.c (完整任务调度器架构)
 * 新增: 跳过第一圈 + 90° 转弯三阶段修复 + 双重去抖 + 计数锁
 * 验证: 180° U-turn 立即触发 + 第二圈全部 90° 转弯 + 无误计线
 *
 * 运行流程:
 *   按键启动 → Line=8, phase=FIRST_LAP → 第1 tick 触发 U-turn
 *      → 180° 掉头 450ms → phase=SECOND_LAP, Line=0
 *      → 第二圈巡线: 0→1(90°)→2(90°)→3→4→5(90°)→6(90°)→7→8
 *      → 永久停车，显示 8
 *
 * 防误判机制 (方案B+D):
 *   方案B 双重去抖:
 *     SM_WAIT_BLACK → 连续全白 8ms 确认 → SM_ON_BLACK
 *     SM_ON_BLACK   → 连续非全白 8ms 确认 → SM_WAIT_WHITE
 *     SM_WAIT_WHITE → 连续全黑 40ms 确认 → 计线
 *   方案D 计数锁:
 *     两次计数间隔 < 250ms → 拒绝计数，重置状态机
 *
 * 与 main.c 的差异:
 *   - 初始 Line=8 / display_line=8（跳过第一圈）
 *   - 90° 转弯三阶段: 冲线→左转→稳定直行（核心修复）
 *   - 双重去抖 + 计数锁防误判（方案B+D）
 * ============================================================ */
#include <reg51.h>
// 第三问：单定时器 + 任务调度器架构
// ISR 仅 PWM + 软件定时器 + sys_tick_flag
// 所有决策逻辑在 main() 任务调度循环中

// ============ 引脚定义 ============
sbit P1_2 = P1^2;
sbit P1_3 = P1^3;
sbit P1_4 = P1^4;
sbit P1_5 = P1^5;
sbit P1_6 = P1^6;
sbit P1_7 = P1^7;
sbit P3_3 = P3^3;
sbit P3_4 = P3^4;
sbit P3_7 = P3^7;

#define uchar unsigned char
#define uint  unsigned int

#define Left_IRSenor_Track   P3_4
#define Right_IRSenor_Track  P3_3

// 速度
#define SPEED               11
#define SPEED_TURN_INNER    5
#define SPEED_TURN_OUTER    6
#define Left_Motor_PWM      P1_4
#define Right_Motor_PWM     P1_5

// 电机方向宏
#define Left_Motor_Go       {P1_2=0; P1_3=1;}
#define Left_Motor_Back     {P1_2=1; P1_3=0;}
#define Left_Motor_Stop     {P1_2=0; P1_3=0;}
#define Right_Motor_Go      {P1_6=1; P1_7=0;}
#define Right_Motor_Back    {P1_6=0; P1_7=1;}
#define Right_Motor_Stop    {P1_6=0; P1_7=0;}

// ============ PWM 变量 ============
unsigned char Left_PWM_Value = 0;
unsigned char Left_Drive_Value = 0;
unsigned char Right_PWM_Value = 0;
unsigned char Right_Drive_Value = 0;
bit Left_moto_stop  = 1;
bit Right_moto_stop = 1;

// ============ 数码管显示 ============
sbit POSSEL = P2^7;
sbit SEGSEL = P2^6;
uchar code POSCode[] = {0xff, 0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};
uchar code SEGCode[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x00
};

// ============ 系统时基 ============
volatile uint sys_tick = 0;
volatile bit sys_tick_flag = 0;

// ============ 任务互斥 ============
bit task_busy = 0;

// ============ 循迹计数 (双重去抖 + 计数锁) ============
uchar Line = 0;
uchar Line_SM = 0;
uchar Debounce_Cnt = 0;       // SM_WAIT_WHITE 去抖计数器
uchar enter_debounce = 0;     // SM_WAIT_BLACK → SM_ON_BLACK 去抖（方案B）
uchar leave_debounce = 0;     // SM_ON_BLACK → SM_WAIT_WHITE 去抖（方案B）
volatile uint last_line_tick = 0;  // 上次计数时的 sys_tick（方案D）

#define DEBOUNCE_COUNT    40   // SM_WAIT_WHITE 去抖阈值（原 DEBOUNCE_MS，加大到 40）
#define DEBOUNCE_ENTER     8   // 进入黑线确认 8ms（方案B 新增）
#define DEBOUNCE_LEAVE     8   // 离开黑线确认 8ms（方案B 新增）
#define MIN_LINE_INTERVAL 250  // 两次计数最小间隔 250ms（方案D 新增）

#define SM_WAIT_BLACK   0
#define SM_ON_BLACK     1
#define SM_WAIT_WHITE   2
#define SM_COUNTED      3

// ============ 阶段 ============
#define FIRST_LAP   0
#define SECOND_LAP  1
uchar phase = FIRST_LAP;
bit task_finished = 0;

// ============ 180° 掉头 ============
volatile bit  uturn_active = 0;
volatile uint uturn_timer = 0;
#define UTURN_DURATION  450

// ============ 90° 转弯 (三阶段修复版) ============
volatile bit  turn90_active = 0;
volatile uint turn90_timer = 0;
uchar turn90_phase = 0;  // 0=冲线直行, 1=左转, 2=稳定直行
#define TURN90_FORWARD      100  // Phase 0 冲线
#define TURN90_DURATION     190  // Phase 1 左转
#define TURN90_STABILIZE    100  // Phase 2 稳定直行（核心修复）
uchar last_turn_line = 0;

// ============ 冷却 ============
volatile bit  cooldown_active = 0;
volatile uint cooldown_timer = 0;
#define COOLDOWN_PERIOD  600

// ============ 显示缓存 ============
uchar display_line = 0;

// ============ 延时函数（仅按键消抖用） ============
void delay_ms(unsigned int n)
{
    unsigned int i, j;
    for(i = n; i > 0; i--)
        for(j = 114; j > 0; j--);
}

// ============ 数码管 ============
void LEDTube_Show(unsigned char digit, unsigned char num)
{
    P0 = POSCode[digit];
    POSSEL = 1;
    POSSEL = 0;
    P0 = SEGCode[num];
    SEGSEL = 1;
    SEGSEL = 0;
}

// ============ 电机控制 ============
void GoForward(void)
{
    Left_Drive_Value  = SPEED;
    Right_Drive_Value = SPEED;
    Left_Motor_Go;
    Right_Motor_Go;
}

void GoLeft(void)
{
    Left_Drive_Value  = SPEED_TURN_INNER;
    Right_Drive_Value = SPEED_TURN_OUTER;
    Left_Motor_Back;
    Right_Motor_Go;
}

void GoRight(void)
{
    Left_Drive_Value  = SPEED_TURN_OUTER;
    Right_Drive_Value = SPEED_TURN_INNER;
    Left_Motor_Go;
    Right_Motor_Back;
}

void Stop(void)
{
    Left_Drive_Value  = 0;
    Right_Drive_Value = 0;
    Left_Motor_Stop;
    Right_Motor_Stop;
}

// ============ PWM 调节 ============
void Left_Motor_PWM_Adjust(void)
{
    if(Left_moto_stop)
    {
        if(Left_PWM_Value <= Left_Drive_Value)
            Left_Motor_PWM = 1;
        else
            Left_Motor_PWM = 0;
        if(Left_PWM_Value >= 20)
            Left_PWM_Value = 0;
    }
    else
        Left_Motor_PWM = 0;
}

void Right_Motor_PWM_Adjust(void)
{
    if(Right_moto_stop)
    {
        if(Right_PWM_Value <= Right_Drive_Value)
            Right_Motor_PWM = 1;
        else
            Right_Motor_PWM = 0;
        if(Right_PWM_Value >= 20)
            Right_PWM_Value = 0;
    }
    else
        Right_Motor_PWM = 0;
}

// ============ 定时器初始化 ============
void TIMER0_Init(void)
{
    TMOD = 0x01;
    TH0 = 0xFC;
    TL0 = 0x18;   // 1ms @12MHz
    TR0 = 1;
    ET0 = 1;
    EA  = 1;
}

// ============================================================
//                    任务函数
// ============================================================

// ---------- Task 1: 传感器 + 电机控制（每 tick） ----------
void Task_SensorMotor(void)
{
    bit left, right;

    if(task_busy) return;

//		if(cooldown_active)
//		{
//			GoForward();
//			return;
//		}
	
    left  = Left_IRSenor_Track;
    right = Right_IRSenor_Track;

    if(left == 0 && right == 0)
        GoForward();
    else if(left == 0 && right == 1)
        GoRight();
    else if(left == 1 && right == 0)
        GoLeft();
    else
        GoForward();
}

// ---------- Task 2: 线计数状态机（双重去抖 + 计数锁） ----------
void Task_LineCounter(void)
{
    bit left, right;

    if(cooldown_active) return;
    if(task_busy)       return;

    left  = Left_IRSenor_Track;
    right = Right_IRSenor_Track;

    switch(Line_SM)
    {
        case SM_WAIT_BLACK:
            // 方案B: 连续全白 DEBOUNCE_ENTER 次才确认进入黑线准备
            if(left == 0 && right == 0)
            {
                enter_debounce++;
                if(enter_debounce >= DEBOUNCE_ENTER)
                {
                    Line_SM = SM_ON_BLACK;
                    enter_debounce = 0;
                }
            }
            else
                enter_debounce = 0;
            break;

        case SM_ON_BLACK:
            // 方案B: 连续非全白 DEBOUNCE_LEAVE 次才确认离开白区
            if(!(left == 0 && right == 0))
            {
                leave_debounce++;
                if(leave_debounce >= DEBOUNCE_LEAVE)
                {
                    Line_SM = SM_WAIT_WHITE;
                    leave_debounce = 0;
                    Debounce_Cnt = 0;
                }
            }
            else
                leave_debounce = 0;
            break;

        case SM_WAIT_WHITE:
            if(left == 1 && right == 1)
            {
                Debounce_Cnt++;
                if(Debounce_Cnt >= DEBOUNCE_COUNT)
                {
                    // 方案D: 计数锁 — 距上次计数不到 MIN_LINE_INTERVAL 则忽略
                    if((sys_tick - last_line_tick) < MIN_LINE_INTERVAL)
                    {
                        // 间隔太近，拒绝计数，重置状态机
                        Line_SM = SM_WAIT_BLACK;
                        Debounce_Cnt = 0;
                        enter_debounce = 0;
                        leave_debounce = 0;
                        cooldown_active = 1;
                        cooldown_timer  = 0;
                        break;
                    }

                    last_line_tick = sys_tick;
                    Line++;
                    display_line = Line;
                    Line_SM = SM_COUNTED;
                    Debounce_Cnt = 0;
                    cooldown_active = 1;
                    cooldown_timer  = 0;
                }
            }
            else
                Debounce_Cnt = 0;
            break;

        case SM_COUNTED:
            break;
    }
}

// ---------- Task 3: 冷却定时器管理（每 tick） ----------
void Task_CooldownTimer(void)
{
    if(cooldown_active && cooldown_timer >= COOLDOWN_PERIOD)
    {
        cooldown_active = 0;
        Line_SM = SM_WAIT_BLACK;
        enter_debounce = 0;
        leave_debounce = 0;
    }
}

// ---------- Task 4: 数码管刷新（每 tick） ----------
void Task_Display(void)
{
    LEDTube_Show(6, display_line);
}

// ---------- Task 5: 180° 掉头（事件触发 + 非阻塞执行） ----------
void Task_UTurn(void)
{
    if(!uturn_active)
    {
        if(task_finished)                 return;
        if(phase != FIRST_LAP)            return;
        if(Line < 8)                      return;
        if(task_busy)                     return;

        uturn_active = 1;
        uturn_timer  = 0;
        task_busy    = 1;
        cooldown_active = 0;  // 方案2+: 转弯 task_busy 已能防止计线，取消冷却
    }

    Left_Drive_Value  = SPEED;
    Right_Drive_Value = SPEED;
    Left_Motor_Back;
    Right_Motor_Go;

    if(uturn_timer >= UTURN_DURATION)
    {
        Stop();
        uturn_active = 0;
        task_busy    = 0;

        phase          = SECOND_LAP;
        Line           = 0;
        display_line   = 0;
        Line_SM        = SM_WAIT_BLACK;
        Debounce_Cnt   = 0;
        enter_debounce = 0;
        leave_debounce = 0;
        last_turn_line = 0;
        // 方案2+: 不重启冷却，转弯后立即恢复计线
    }
}

// ---------- Task 6: 90° 转弯（三阶段修复版） ----------
void Task_Turn90(void)
{
    if(!turn90_active)
    {
        if(task_finished)                            return;
        if(phase != SECOND_LAP)                      return;
        if(task_busy)                                return;
        if(!(Line == 1 || Line == 2 ||
             Line == 5 || Line == 6))                return;
        if(Line == last_turn_line)                   return;

        turn90_active = 1;
        turn90_timer  = 0;
        turn90_phase  = 0;
        task_busy     = 1;
        last_turn_line = Line;
        cooldown_active = 0;  // 方案2+: 转弯 task_busy 已能防止计线，取消冷却
    }

    if(turn90_phase == 0)
    {
        // Phase 0: 冲线直行 120ms
        Left_Drive_Value  = SPEED-2;
        Right_Drive_Value = SPEED-2;
        Left_Motor_Go;
        Right_Motor_Go;

        if(turn90_timer >= TURN90_FORWARD)
        {
            turn90_phase = 1;
            turn90_timer = 0;
        }
    }
    else if(turn90_phase == 1)
    {
        // Phase 1: 原地左转 300ms
        Left_Drive_Value  = SPEED;
        Right_Drive_Value = SPEED;
        Left_Motor_Back;
        Right_Motor_Go;

        if(turn90_timer >= TURN90_DURATION)
        {
            turn90_phase = 2;
            turn90_timer = 0;
        }
    }
    else
    {
        // Phase 2: 稳定直行 150ms（核心修复）
        Left_Drive_Value  = SPEED-2;
        Right_Drive_Value = SPEED-2;
        Left_Motor_Go;
        Right_Motor_Go;

        if(turn90_timer >= TURN90_STABILIZE)
        {
            Stop();
            turn90_active = 0;
            task_busy     = 0;

            Line_SM      = SM_WAIT_BLACK;
            Debounce_Cnt = 0;
            enter_debounce = 0;
            leave_debounce = 0;
            // 方案2+: 不重启冷却，转弯后立即恢复计线
        }
    }
}

// ---------- Task 7: 阶段管理 ----------
void Task_PhaseManager(void)
{
    if(task_finished) return;
    if(phase == SECOND_LAP && Line >= 8)
    {
        Stop();
        task_finished = 1;
        task_busy     = 1;
        display_line  = Line;
    }
}

// ============================================================
//                    主函数（★ 测试差异点）
// ============================================================
void main(void)
{
    P1 = 0xF0;

    // ---- 等待按键启动 ----
    while(1)
    {
        if(P3_7 != 1)
        {
            delay_ms(50);
            if(P3_7 != 1) break;
        }
    }
    delay_ms(50);
    TIMER0_Init();

    // ★ 差异点：跳过第一圈，直接以 Line=8 启动
    Line = 8;
    phase = FIRST_LAP;
    display_line = 8;

    // ---- 主调度循环（与 main.c 完全相同） ----
    while(1)
    {
        if(!sys_tick_flag) continue;
        sys_tick_flag = 0;

        Task_UTurn();
        Task_Turn90();
        Task_PhaseManager();
        Task_SensorMotor();
        Task_LineCounter();
        Task_CooldownTimer();
        Task_Display();
    }
}

// ============================================================
//               Timer 0 中断服务程序（极简）
// ============================================================
void TIMER0_IRQHandler(void) interrupt 1 using 2
{
    TH0 = 0xFC;
    TL0 = 0x18;

    Left_PWM_Value++;
    Right_PWM_Value++;
    Left_Motor_PWM_Adjust();
    Right_Motor_PWM_Adjust();

    if(cooldown_active) cooldown_timer++;
    if(uturn_active)    uturn_timer++;
    if(turn90_active)   turn90_timer++;

    sys_tick++;
    sys_tick_flag = 1;
}
