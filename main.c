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
#define SPEED_TURN_INNER    4
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
unsigned char Left_PWM_Value = 0;//PWM值
unsigned char Left_Drive_Value = 0;//驱动界限值
unsigned char Right_PWM_Value = 0;//PWM值
unsigned char Right_Drive_Value = 0;//驱动界限值
bit Left_moto_stop  = 1;//左电机使能位
bit Right_moto_stop = 1;//右电机使能位

// ============ 数码管显示 ============
sbit POSSEL = P2^7;
sbit SEGSEL = P2^6;
uchar code POSCode[] = {0xff, 0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};
uchar code SEGCode[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x00
};

// ============ 系统时基 ============
volatile uint sys_tick = 0;//系统滴答时基
volatile bit sys_tick_flag = 0;

// ============ 任务互斥 ============
bit task_busy = 0;//互斥锁

// ============ 循迹计数 (方案B 三重去抖 + 方案D 计数锁) ============
uchar Line = 0;
uchar Line_SM = 0;             // 初始 = SM_WAIT_BLACK
uchar Debounce_Cnt = 0;        // SM_WAIT_WHITE 去抖计数器
uchar enter_debounce = 0;      // SM_WAIT_BLACK → SM_ON_BLACK 去抖
uchar leave_debounce = 0;      // SM_ON_BLACK → SM_WAIT_WHITE 去抖
volatile uint last_line_tick = 0;  // 上次计数时的 sys_tick（方案D 计数锁）

#define DEBOUNCE_COUNT    40   // SM_WAIT_WHITE 去抖阈值（原20，加大到40）
#define DEBOUNCE_ENTER     8   // 进入黑线确认 8ms
#define DEBOUNCE_LEAVE     8   // 离开黑线确认 8ms
#define MIN_LINE_INTERVAL 380  // 两次计数最小间隔 250ms

#define SM_WAIT_BLACK   0//状态机：等待黑线
#define SM_ON_BLACK     1//状态机：在黑线上
#define SM_WAIT_WHITE   2//状态机：等待白线
#define SM_COUNTED      3//状态机：计次完成

// ============ 阶段 ============
#define FIRST_LAP   0//第1圈
#define SECOND_LAP  1//第2圈
uchar phase = FIRST_LAP;//阶段：初始值为第1圈
bit task_finished = 0;//任务完成使能位

// ============ 180° 掉头 ============
volatile bit  uturn_active = 0;//180°转向激活位
volatile uint uturn_timer = 0;//180°转向计时器
#define UTURN_DURATION  447//转向延时时间参数

// ============ 90° 转弯 ============
volatile bit  turn90_active = 0;//90°转向激活位
volatile uint turn90_timer = 0;//90°转向计时器
uchar turn90_phase = 0;  // 0=短直行冲线, 1=转弯中
#define TURN90_FORWARD  150//90°冲线时间参数
#define TURN90_DURATION 240//90°转弯时间参数
#define TURN90_PAUSE       300   // Phase 2 短暂停车，消除旋转惯性
#define TURN90_STABILIZE    125  // Phase 3 稳定直行
uchar last_turn_line = 0;//最近一次转弯线

// ============ 冷却 ============
volatile bit  cooldown_active = 0;//冷却激活位
volatile uint cooldown_timer = 0;//冷却计数器
#define COOLDOWN_PERIOD  400//冷却延时时间参数

// ============ 显示缓存 ============
uchar display_line = 0;//显示缓存

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
void GoForward(void)//直行
{
    Left_Drive_Value  = SPEED;//左侧驱动占空比
    Right_Drive_Value = SPEED;//右侧驱动占空比
		Left_Motor_Go;//左电机直行
		Right_Motor_Go;//右电机直行
}

void GoLeft(void)//左转
{
    Left_Drive_Value  = SPEED_TURN_INNER;//左侧驱动占空比：转弯内侧的速度
    Right_Drive_Value = SPEED_TURN_OUTER;//右侧驱动占空比：转弯外侧的速度
		Left_Motor_Back;//左电机后退
		Right_Motor_Go;//右电机直行
}

void GoRight(void)
{
    Left_Drive_Value  = SPEED_TURN_OUTER;//左侧驱动占空比：转弯外侧的速度
    Right_Drive_Value = SPEED_TURN_INNER;//右侧驱动占空比：转弯内侧的速度
		Left_Motor_Go;//左电机直行
		Right_Motor_Back;//右电机后退
}

void Stop(void)
{
	Left_Drive_Value  = 0;//左电机驱动占空比为0
	Right_Drive_Value = 0;//右电机驱动占空比为0
	Left_Motor_Stop;//左电机停止
	Right_Motor_Stop;//右电机停止
}

// ============ PWM 调节 ============
void Left_Motor_PWM_Adjust(void)//左电机PWM调节
{
	if(Left_moto_stop)//如果左电机使能
    {
			if(Left_PWM_Value <= Left_Drive_Value)//如果左电机PWM值<=驱动驱动界限值
				Left_Motor_PWM = 1;//左电机使能
        else
					Left_Motor_PWM = 0;//左电机不使能
        if(Left_PWM_Value >= 20)//超出最大PWM周期
					Left_PWM_Value = 0;//左电机不使能
    }
    else
			Left_Motor_PWM = 0;//其他情况左电机不使能
}

void Right_Motor_PWM_Adjust(void)//右电机PWM调节
{
    if(Right_moto_stop)//如果右电机使能
    {
        if(Right_PWM_Value <= Right_Drive_Value)//如果右电机PWM值<=驱动驱动界限值
            Right_Motor_PWM = 1;//右电机使能
        else
            Right_Motor_PWM = 0;//右电机不使能
        if(Right_PWM_Value >= 20)//超出最大PWM周期
            Right_PWM_Value = 0;//右电机不使能
    }
    else
        Right_Motor_PWM = 0;//其他情况右电机不使能
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
// 动作任务期间跳过，由动作任务自己控制电机
void Task_SensorMotor(void)//传感器加电机控制任务函数
{
    bit left, right;//左右传感器接收位

    if(task_busy) return;//如果CPU被占用，退出任务函数

    left  = Left_IRSenor_Track;//左循迹传感器返回值
    right = Right_IRSenor_Track;//右循迹传感器返回值

	if(left == 0 && right == 0)//如果左右都为0，即都是白色
        GoForward();//直行
    else if(left == 0 && right == 1)//如果左侧白色，右侧黑色
        GoRight();//右转
    else if(left == 1 && right == 0)//如果右侧白色，左侧黑色
        GoLeft();//左转
    else//其他情况
        GoForward();//直行
}

// ---------- Task 2: 线计数状态机（方案B 三重去抖 + 方案D 计数锁） ----------
void Task_LineCounter(void)//计线任务函数
{
    bit left, right;//左右传感器返回值接收位

    if(cooldown_active) return;//如果冷却函数激活，退出
    if(task_busy)       return;//如果CPU占用，退出

    left  = Left_IRSenor_Track;//读取左传感器
    right = Right_IRSenor_Track;//读取右传感器

	switch(Line_SM)//Line_SM计线状态
    {
        case SM_WAIT_BLACK://等待黑线状态
            // 方案B: 连续全白 DEBOUNCE_ENTER 次才确认
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

        case SM_ON_BLACK://在黑线上状态
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

        case SM_WAIT_WHITE://在白线上状态
            if(left == 1 && right == 1)
            {
                Debounce_Cnt++;//去除抖动计数器
				if(Debounce_Cnt >= DEBOUNCE_COUNT)//如果去除抖动计数次数>=去抖时间
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
                    Line++;//线数+1
					display_line = Line;//将线数赋值给显示值
                    Line_SM = SM_COUNTED;//
                    Debounce_Cnt = 0;
                    cooldown_active = 1;
                    cooldown_timer = 0;
                }
            }
            else
                Debounce_Cnt = 0;
            break;

        case SM_COUNTED:
            // 等待冷却结束，由 Task_CooldownTimer 切回 SM_WAIT_BLACK
            break;
    }
}

// ---------- Task 3: 冷却定时器管理（每 tick） ----------
void Task_CooldownTimer(void)//冷却定时器
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
    // ---- 触发条件检查 ----
    if(!uturn_active)
    {
        if(task_finished)                 return;
        if(phase != FIRST_LAP)            return;
        if(Line < 8)                      return;
        if(task_busy)                     return;

        // 触发掉头
        uturn_active = 1;
        uturn_timer  = 0;
        task_busy    = 1;
        cooldown_active = 0;  // 方案2+: 转弯 task_busy 已能防止计线，取消冷却
        // 直接进入执行（fall through）
    }

    // ---- 执行：左后 + 右前 = 顺时针原地旋转 ----
    Left_Drive_Value  = SPEED;
    Right_Drive_Value = SPEED;
    Left_Motor_Back;
    Right_Motor_Go;

    // ---- 完成检查（ISR 每 tick 递增 uturn_timer） ----
    if(uturn_timer >= UTURN_DURATION)
    {
        Stop();
        uturn_active = 0;
        task_busy    = 0;

        // 切换到第二圈
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

// ---------- Task 6: 90° 转弯（事件触发 + 非阻塞执行） ----------
void Task_Turn90(void)
{
    // ---- 触发条件检查 ----
    if(!turn90_active)
    {
        if(task_finished)                            return;
        if(phase != SECOND_LAP)                      return;
        if(task_busy)                                return;
        if(!(Line == 1 || Line == 2 ||
             Line == 5 || Line == 6))                return;
        if(Line == last_turn_line)                   return;

        // 触发转弯
        turn90_active = 1;
        turn90_timer  = 0;
        turn90_phase  = 0;   // 先刹车减速
        task_busy     = 1;
        last_turn_line = Line;
        cooldown_active = 0;  // 方案2+: 转弯 task_busy 已能防止计线，取消冷却
        // 直接进入执行（fall through）
    }

    // ---- 执行状态机 ----
    if(turn90_phase == 0)
    {
        if(last_turn_line == 1||last_turn_line == 2||last_turn_line == 5||last_turn_line == 6)
        {
            // Line=6: 速度太快，刹车减速
            Left_Drive_Value  = 2;
            Right_Drive_Value = 2;
            Left_Motor_Go;
            Right_Motor_Go;
        }
//        else
//        {
//            // Line=1,2,5: 正常冲线直行
//            Left_Drive_Value  = SPEED-2;
//            Right_Drive_Value = SPEED-2;
//            Left_Motor_Go;
//            Right_Motor_Go;
//        }

        if(turn90_timer >= TURN90_FORWARD)
        {
            turn90_phase = 1;
            turn90_timer = 0;
        }
    }
    else if(turn90_phase == 1)
    {
        // Phase 1: 原地左转
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
    else if(turn90_phase == 2)
    {
        // Phase 2: 短暂停车 — 消除旋转惯性
        Stop();

        if(turn90_timer >= TURN90_PAUSE)
        {
            turn90_phase = 3;
            turn90_timer = 0;
        }
    }
    else  // turn90_phase == 3
    {
        // Phase 3: 稳定直行 → 完成后立即交还传感器，不锁死方向
        Left_Drive_Value  = SPEED;
        Right_Drive_Value = SPEED;
        Left_Motor_Go;
        Right_Motor_Go;

        if(turn90_timer >= TURN90_STABILIZE)
        {
            Stop();
            turn90_active = 0;
            task_busy     = 0;

            // 重置计数状态机
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
        task_busy     = 1;  // 禁止一切后续动作
        display_line  = Line;  // 保持显示 8
    }
}

// ============================================================
//                    主函数
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

    display_line = Line;  // 初始显示 0

    // ---- 主调度循环 ----
    while(1)
    {
        // 等待 1ms 节拍
			if(!sys_tick_flag) continue;//如果滴答标志位为0，继续后面任务调度
        sys_tick_flag = 0;//滴答时钟标志位置0

        // 1. 动作任务（含触发 + 非阻塞执行）
        Task_UTurn();//180°转弯
        Task_Turn90();//90°转弯
        Task_PhaseManager();//阶段管理

        // 2. 常规任务
			Task_SensorMotor();//传感器加电机任务
        Task_LineCounter();//计线器任务
        Task_CooldownTimer();//冷却定时器任务函数
			Task_Display();//显示任务函数
    }
}

// ============================================================
//               Timer 0 中断服务程序（极简）
// ============================================================
void TIMER0_IRQHandler(void) interrupt 1 using 2
{
    TH0 = 0xFC;//高八位重装载值
    TL0 = 0x18;//第八位重装载值

    // 1. PWM
	Left_PWM_Value++;//左电机PWM值自增
	Right_PWM_Value++;//右电机PWM值自增
	Left_Motor_PWM_Adjust();//左电机PWM判断
	Right_Motor_PWM_Adjust();//右电机PWM判断

    // 2. 软件定时器
    if(cooldown_active) cooldown_timer++;//如果处于激活态，冷却函数计时+1
    if(uturn_active)    uturn_timer++;//如果处于激活态，180°转弯函数计时+1
    if(turn90_active)   turn90_timer++;//如果处于激活态，90°转弯函数计时+1

    // 3. 系统时基标志
    sys_tick++;//滴答自增
    sys_tick_flag = 1;//滴答标志位置1
}
