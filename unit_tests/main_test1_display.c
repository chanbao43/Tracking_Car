/* ============================================================
 * Test 1: 两位数码管显示
 * 基于: 原始 180° 掉头版本
 * 新增: 数码管改为两位显示（位置1=十位, 位置2=个位）
 * 验证: 计数 0→1→...→8→0→1... 数码管正确显示
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

bit  U_Turn_Active = 0;
unsigned int U_Turn_Timer = 0;
#define U_TURN_DURATION    600
uchar Cooldown_Cnt = 0;
#define COOLDOWN_MS       100

void delay_ms(unsigned int n)
{
    unsigned int i, j;
    for(i = n; i > 0; i--)
        for(j = 114; j > 0; j--);
}

void LEDTube_Show(unsigned char i, unsigned char j)
{
    P0 = POSCode[i];
    POSSEL = 1;
    POSSEL = 0;
    P0 = SEGCode[j];
    SEGSEL = 1;
    SEGSEL = 0;
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
        // [NEW] 两位数码管显示（十位 + 个位）
        LEDTube_Show(1, Line / 10);
        LEDTube_Show(2, Line % 10);

        // 计满 8 条线，触发掉头
        if(Line >= 8)
        {
            U_Turn_Active = 1;
            U_Turn_Timer = 0;
            Line = 0;
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

    if(U_Turn_Active == 0)
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
                }
            }
            else if(Black_Line_Detected == 0)
            {
                Debounce_Cnt = 0;
            }
        }

        if(Line >= 8)
        {
            U_Turn_Active = 1;
            U_Turn_Timer = 0;
            Line = 0;
            Black_Line_Detected = 0;
            Counting_Ready = 0;
            Debounce_Cnt = 0;
            Cooldown_Cnt = 0;
        }
    }
    else
    {
        Left_Drive_Value  = SPEED;
        Right_Drive_Value = SPEED;
        Left_Motor_Go;
        Right_Motor_Back;

        U_Turn_Timer++;
        if(U_Turn_Timer >= U_TURN_DURATION)
        {
            U_Turn_Active = 0;
        }
    }
}
