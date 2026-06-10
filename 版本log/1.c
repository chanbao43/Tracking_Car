#include <reg51.h>
//已经实现第二问
// ????(?? HJduino ????)
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

// ?????????
#define Left_IRSenor_Track   P3_4  
#define Right_IRSenor_Track  P3_3  

// ??????
#define SPEED               9
#define SPEED_TURN_INNER    5     // 转弯内侧轮速度（慢）
#define SPEED_TURN_OUTER    5     // 转弯外侧轮速度（较快，仍慢于直行）
#define Left_Motor_PWM	     P1_4	 
#define Right_Motor_PWM	     P1_5	 

// ?????(?? L298N ???)
#define Left_Motor_Go      {P1_2=0; P1_3=1;} 
#define Left_Motor_Back    {P1_2=1; P1_3=0;} 
#define Left_Motor_Stop    {P1_2=0; P1_3=0;}                      
#define Right_Motor_Go     {P1_6=1; P1_7=0;}	
#define Right_Motor_Back   {P1_6=0; P1_7=1;}	
#define Right_Motor_Stop   {P1_6=0; P1_7=0;}	

// PWM ????
unsigned char Left_PWM_Value = 0;
unsigned char Left_Drive_Value = 0;
unsigned char Right_PWM_Value = 0;
unsigned char Right_Drive_Value = 0;
bit Left_moto_stop  = 1;
bit Right_moto_stop = 1;

// ??????????
sbit POSSEL = P2^7;
sbit SEGSEL = P2^6;
uchar POSCode[] = {0xff, 0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};
uchar code SEGCode[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x00
};

// ??????
unsigned char Line = 0;            // ?????????
uchar Debounce_Cnt = 0;            // ?????

// 线计数状态机（严格序列：黑→离开→白确认→计数→冷却）
#define SM_WAIT_BLACK   0          // 等待进入黑线
#define SM_ON_BLACK     1          // 已检测到黑线（在黑线上）
#define SM_WAIT_WHITE   2          // 已离开黑线，等待白线确认
#define SM_COUNTED      3          // 已计数，冷却中
uchar Line_SM = SM_WAIT_BLACK;

bit  U_Turn_Active = 0;            // 180° 掉头进行中
bit  Turn_90_Active = 0;           // 90° 转弯进行中
bit  Stop_Flag = 0;                // 停车标志（持久）
bit  Turn_Flag = 0;                // 0=第一段, 1=第二段(掉头后)
unsigned int U_Turn_Timer = 0;     // 掉头计时器 (ms)
unsigned int Turn_90_Timer = 0;    // 90° 转弯计时器 (ms)
#define U_TURN_DURATION    600     // 掉头持续 600ms
#define TURN_90_DURATION   140     // 90° 转弯 140ms
uchar Cooldown_Cnt = 0;            // 冷却计数器
#define COOLDOWN_MS       100      // 冷却期 100ms

// ????(? n ??)
void delay_ms(unsigned int n)
{
    unsigned int i, j;
    for(i = n; i > 0; i--)
        for(j = 114; j > 0; j--);
}

// ?????(?? i ????? j)
void LEDTube_Show(unsigned char i, unsigned char j)
{
    P0 = POSCode[i];
    POSSEL = 1;
    POSSEL = 0;
    P0 = SEGCode[j];
    SEGSEL = 1;
    SEGSEL = 0;
}

// ????????
void GoForward(void)
{
    Left_Drive_Value  = SPEED;	 
    Right_Drive_Value = SPEED; 
    Left_Motor_Go;  
    Right_Motor_Go; 
}

void GoLeft(void)
{
    Left_Drive_Value  = SPEED_TURN_INNER;   // 左轮慢（内侧）
    Right_Drive_Value = SPEED_TURN_OUTER;   // 右轮快（外侧）
    Left_Motor_Back;    // 
    Right_Motor_Go;
}

void GoRight(void)
{
    Left_Drive_Value  = SPEED_TURN_OUTER;   // 左轮快（外侧）
    Right_Drive_Value = SPEED_TURN_INNER;   // 右轮慢（内侧）
    Left_Motor_Go;    // 
    Right_Motor_Back;
}

void Stop(void)	    
{	 
    Left_Drive_Value  = 0;      // ??????
    Right_Drive_Value = 0;
    Left_Motor_Stop;    
    Right_Motor_Stop;   
}

// ??? PWM ??(? 1ms ????)
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
    {
        Left_Motor_PWM = 0;
    }
}

// ??? PWM ??(? 1ms ????)
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
    {
        Right_Motor_PWM = 0;
    }	
}

// ??? 0 ???(1ms ??)
void TIMER0_Init(void)
{
    TMOD = 0x01;       
    TH0 = 0xFC; 
    TL0 = 0x18;     	
    TR0 = 1;   
    ET0 = 1;	  
    EA  = 1;	   
}

// ???
void main(void)
{
    P1 = 0xF0;   // ??? P1 ????????
	
    // ????????(P3_7 ?????)
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
	
    while(1)
    {
        // ????? 6 ??????????
        LEDTube_Show(6, Line);

        // 第一段计满 8 条线，触发 180° 掉头（不重置 Line，继续数）
        if(Turn_Flag == 0 && Line >= 8)
        {
//            U_Turn_Active = 1;
//            U_Turn_Timer = 0;
            Line_SM = SM_WAIT_BLACK;
            Debounce_Cnt = 0;
            Cooldown_Cnt = 0;
        }
    }
}

// ??? 0 ??????(1ms ??,?? PWM ?? + ????)
void TIMER0_IRQHandler(void) interrupt 1 using 2
{
    // ????,?? 1ms ????
    TH0 = 0xFC;	  
    TL0 = 0x18;
    
    // 1. ?? PWM ??(??? 20,?? 20ms)
    Left_PWM_Value++; 
    Right_PWM_Value++;
    Left_Motor_PWM_Adjust();
    Right_Motor_PWM_Adjust();
    
    // ========== 状态路由 ==========
    if(Stop_Flag == 1)
    {
        Stop();                      // 持续停车，不再循迹
    }
    else if(U_Turn_Active == 1)
    {
        // ── 180° 原地自旋 ──
        Left_Drive_Value  = SPEED;
        Right_Drive_Value = SPEED;
        Left_Motor_Go;
        Right_Motor_Back;

        U_Turn_Timer++;
        if(U_Turn_Timer >= U_TURN_DURATION)
        {
            U_Turn_Active = 0;
            Turn_Flag = 1;              // 进入第二段
        }
    }
    else if(Turn_90_Active == 1&&Stop_Flag == 0)
    {
        // ── 90° 左转（原地坦克转向）──
        Left_Drive_Value  = SPEED;
        Right_Drive_Value = SPEED;
        Left_Motor_Back;
        Right_Motor_Go;

        Turn_90_Timer++;
        if(Turn_90_Timer >= TURN_90_DURATION)
        {
            Turn_90_Active = 0;
            Turn_90_Timer = 0;
            Stop_Flag = 1;              // 持久停车
            Stop();
        }
    }
    else
    {
        // ── 正常循迹 ──
        bit left  = Left_IRSenor_Track;
        bit right = Right_IRSenor_Track;

        // ---------- 电机控制 ----------
        if(left == 0 && right == 0)       // 双黑：直行
        {
            GoForward();
        }
        else if(left == 0 && right == 1)  // 左黑右白：右转修正
        {
            GoRight();
        }
        else if(left == 1 && right == 0)  // 左白右黑：左转修正
        {
            GoLeft();
        }
        else                               // 双白：直行
        {
            GoForward();
        }

        // ---------- 状态机计数 ----------
        switch(Line_SM)
        {
            case SM_WAIT_BLACK:            // 等待进入黑线
                if(left == 0 && right == 0)
                    Line_SM = SM_ON_BLACK;
                break;

            case SM_ON_BLACK:              // 在黑线上，等待离开
                if(!(left == 0 && right == 0))
                {
                    Line_SM = SM_WAIT_WHITE;
                    Debounce_Cnt = 0;
                }
                break;

            case SM_WAIT_WHITE:            // 已离开，去抖确认白线
                if(left == 1 && right == 1)
                {
                    Debounce_Cnt++;
                    if(Debounce_Cnt >= 20)
                    {
                        Line++;
                        Line_SM = SM_COUNTED;
                        Cooldown_Cnt = COOLDOWN_MS;
                        Debounce_Cnt = 0;

                        // ── 计数后检查特殊事件 ──
//                        if(Turn_Flag == 0 && Line >= 8)
//                        {
//                            // 第一段到 8：触发 180° 掉头
//                            U_Turn_Active = 1;
//                            U_Turn_Timer = 0;
//                        }
                         if(Turn_Flag == 1 && Line == 8)
                        {
                            // 第二段到 11：触发 90° 左转
                            Turn_90_Active = 1;
                            Turn_90_Timer = 0;
                        }
                    }
                }
                else                       // 非双白，复位去抖
                {
                    Debounce_Cnt = 0;
                }
                break;

            case SM_COUNTED:               // 已计数，冷却中
                if(Cooldown_Cnt > 0)
                    Cooldown_Cnt--;
                else
                    Line_SM = SM_WAIT_BLACK;
                break;
        }
    }
}