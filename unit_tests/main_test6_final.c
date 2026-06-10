/* ============================================================
 * Test 6: 完整版 + 消影
 * 基于: Test 5 (多线号 90° 转弯 + 防重复)
 * 新增: LEDTube_Show 消影处理
 * 验证: 完整功能 + 数码管无鬼影
 *
 * 完整运行流程:
 *   起点 → 1→2→...→8 → [180°掉头] → 9→10→11→12→13→14→15→16 → [STOP]
 *                         Turn_Flag=1    90° 90°          90° 90°
 * ============================================================ */
#include <reg51.h>

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

#define SPEED               9
#define SPEED_TURN_INNER    4
#define SPEED_TURN_OUTER    5
#define Left_Motor_PWM      P1_4
#define Right_Motor_PWM     P1_5

#define Left_Motor_Go      {P1_2=0; P1_3=1;}
#define Left_Motor_Back    {P1_2=1; P1_3=0;}
#define Left_Motor_Stop    {P1_2=0; P1_3=0;}
#define Right_Motor_Go     {P1_6=1; P1_7=0;}
#define Right_Motor_Back   {P1_6=0; P1_7=1;}
#define Right_Motor_Stop   {P1_6=0; P1_7=0;}

unsigned char Left_PWM_Value = 0;
unsigned char Left_Drive_Value = 0;
unsigned char Right_PWM_Value = 0;
unsigned char Right_Drive_Value = 0;
bit Left_moto_stop  = 1;
bit Right_moto_stop = 1;

sbit POSSEL = P2^7;
sbit SEGSEL = P2^6;
uchar POSCode[] = {0xff, 0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};
uchar code SEGCode[] = {
    0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07,
    0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71, 0x00
};

// ========== 循迹变量 ==========
unsigned char Line = 0;
bit Black_Line_Detected = 0;
bit Counting_Ready = 0;
uchar Debounce_Cnt = 0;

// 掉头与 90° 转弯状态
bit  U_Turn_Active = 0;
unsigned int U_Turn_Timer = 0;
#define U_TURN_DURATION    600

bit  Turn_90_Active = 0;
unsigned int Turn_90_Timer = 0;
#define TURN_90_DURATION   140

bit  Turn_Flag = 0;
bit  Special_Turn_Done = 0;

uchar Cooldown_Cnt = 0;
#define COOLDOWN_MS       100

void delay_ms(unsigned int n)
{
    unsigned int i, j;
    for(i = n; i > 0; i--)
        for(j = 114; j > 0; j--);
}

// [NEW] 消影版 LEDTube_Show
void LEDTube_Show(unsigned char i, unsigned char j)
{
    // 消影：先关闭所有位选
    P0 = 0xff;
    POSSEL = 1;
    POSSEL = 0;

    // 设置段选数据
    P0 = SEGCode[j];
    SEGSEL = 1;
    SEGSEL = 0;

    // 打开目标位选
    P0 = POSCode[i];
    POSSEL = 1;
    POSSEL = 0;
}

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

void TIMER0_Init(void)
{
    TMOD = 0x01;
    TH0 = 0xFC;
    TL0 = 0x18;
    TR0 = 1;
    ET0 = 1;
    EA  = 1;
}

void main(void)
{
    P1 = 0xF0;

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
        LEDTube_Show(1, Line / 10);
        LEDTube_Show(2, Line % 10);

        if(Turn_Flag == 1 && Line >= 16)
        {
            Stop();
        }

        if(Turn_Flag == 0 && Line >= 8)
        {
            U_Turn_Active = 1;
            U_Turn_Timer = 0;
            Black_Line_Detected = 0;
            Counting_Ready = 0;
            Debounce_Cnt = 0;
            Cooldown_Cnt = 0;
        }
    }
}

void TIMER0_IRQHandler(void) interrupt 1 using 2
{
    TH0 = 0xFC;
    TL0 = 0x18;

    Left_PWM_Value++;
    Right_PWM_Value++;
    Left_Motor_PWM_Adjust();
    Right_Motor_PWM_Adjust();

    // ========== 状态路由 ==========
    if(U_Turn_Active == 1)
    {
        Left_Drive_Value  = SPEED;
        Right_Drive_Value = SPEED;
        Left_Motor_Go;
        Right_Motor_Back;

        U_Turn_Timer++;
        if(U_Turn_Timer >= U_TURN_DURATION)
        {
            U_Turn_Active = 0;
            Turn_Flag = 1;
        }
    }
    else if(Turn_90_Active == 1)
    {
        Left_Drive_Value  = SPEED;
        Right_Drive_Value = SPEED;
        Left_Motor_Back;
        Right_Motor_Go;

        Turn_90_Timer++;
        if(Turn_90_Timer >= TURN_90_DURATION)
        {
            Turn_90_Active = 0;
            Turn_90_Timer = 0;
            Black_Line_Detected = 0;
            Counting_Ready = 0;
            Debounce_Cnt = 0;
            Cooldown_Cnt = 0;
        }
    }
    else
    {
        bit left  = Left_IRSenor_Track;
        bit right = Right_IRSenor_Track;

        if(left == 0 && right == 0)
        {
            GoForward();
            Black_Line_Detected = 1;
            Counting_Ready = 0;
            Debounce_Cnt = 0;
        }
        else if(left == 0 && right == 1)
        {
            GoRight();
            Counting_Ready = 0;
            Debounce_Cnt = 0;
        }
        else if(left == 1 && right == 0)
        {
            GoLeft();
            Counting_Ready = 0;
            Debounce_Cnt = 0;
        }
        else if(left == 1 && right == 1)
        {
            GoForward();

            if(Cooldown_Cnt > 0)
            {
                Cooldown_Cnt--;
                Debounce_Cnt = 0;
            }
            else if(Black_Line_Detected == 1 && Counting_Ready == 0)
            {
                Debounce_Cnt++;
                if(Debounce_Cnt >= 20)
                {
                    Line++;
                    Black_Line_Detected = 0;
                    Counting_Ready = 1;
                    Cooldown_Cnt = COOLDOWN_MS;
                    Debounce_Cnt = 0;

                    if(Turn_Flag == 0 && Line >= 8)
                    {
                        U_Turn_Active = 1;
                        U_Turn_Timer = 0;
                    }
                    else if(Turn_Flag == 1 && Special_Turn_Done == 0)
                    {
                        if(Line == 10 || Line == 11 || Line == 14 || Line == 15)
                        {
                            Turn_90_Active = 1;
                            Turn_90_Timer = 0;
                            Special_Turn_Done = 1;
                        }
                    }
                    if(Line != 10 && Line != 11 && Line != 14 && Line != 15)
                    {
                        Special_Turn_Done = 0;
                    }
                }
            }
            else if(Black_Line_Detected == 0)
            {
                Debounce_Cnt = 0;
            }
        }
    }
}
