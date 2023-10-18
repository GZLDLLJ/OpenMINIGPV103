/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*
 *@Note
 USART Print debugging routine:
 USART1_Tx(PA9).
 This example demonstrates using USART1(PA9) as a print debug port output.

*/

#include "debug.h"
#include "multi_button.h"
#include "key.h"

/* Global typedef */
enum Button_IDs {
    btn1_id,
    btn2_id,
    btn3_id,
    btn4_id,
    btn5_id,
    btn6_id,
    btn7_id,
    btn8_id,
    btn9_id,
};

struct Button btn1;
struct Button btn2;
struct Button btn3;
struct Button btn4;
struct Button btn5;
struct Button btn6;
struct Button btn7;
struct Button btn8;
struct Button btn9;
/* Global define */

/* Global Variable */



uint8_t read_button_GPIO(uint8_t button_id)
{
    // you can share the GPIO read function with multiple Buttons
    switch(button_id)
    {
        case btn1_id:
            return GPIO_ReadInputDataBit(B1_GPIO_PORT, B1_PIN);
        case btn2_id:
            return GPIO_ReadInputDataBit(B2_GPIO_PORT, B2_PIN);
        case btn3_id:
            return GPIO_ReadInputDataBit(B3_GPIO_PORT, B3_PIN);
        case btn4_id:
            return GPIO_ReadInputDataBit(B4_GPIO_PORT, B4_PIN);
        case btn5_id:
            return GPIO_ReadInputDataBit(B5_GPIO_PORT, B5_PIN);
        case btn6_id:
            return GPIO_ReadInputDataBit(B6_GPIO_PORT, B6_PIN);
        case btn7_id:
            return GPIO_ReadInputDataBit(B7_GPIO_PORT, B7_PIN);
        case btn8_id:
            return GPIO_ReadInputDataBit(B8_GPIO_PORT, B8_PIN);
        case btn9_id:
            return GPIO_ReadInputDataBit(B9_GPIO_PORT, B9_PIN);
        default:
            return 0;
    }
}



void BTN1_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN1_PRESS_DOWN\r\n");
}

void BTN1_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN1_PRESS_UP\r\n");
}

void BTN2_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN2_PRESS_DOWN\r\n");
}

void BTN2_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN2_PRESS_UP\r\n");
}
void BTN3_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN3_PRESS_DOWN\r\n");
}

void BTN3_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN3_PRESS_UP\r\n");
}
void BTN4_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN4_PRESS_DOWN\r\n");
}

void BTN4_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN4_PRESS_UP\r\n");
}
void BTN5_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN5_PRESS_DOWN\r\n");
}

void BTN5_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN5_PRESS_UP\r\n");
}

void BTN6_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN6_PRESS_DOWN\r\n");
}

void BTN6_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN6_PRESS_UP\r\n");
}
void BTN7_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN7_PRESS_DOWN\r\n");
}

void BTN7_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN7_PRESS_UP\r\n");
}
void BTN8_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN8_PRESS_DOWN\r\n");
}

void BTN8_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN8_PRESS_UP\r\n");
}

void BTN9_PRESS_DOWN_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN9_PRESS_DOWN\r\n");
}

void BTN9_PRESS_UP_Handler(void* btn)
{
    //do something...
    printf("\r\nBTN9_PRESS_UP\r\n");
}
/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);

    printf("This is printf example\r\n");

    KEY_INIT();
    button_init(&btn1, read_button_GPIO, 1, btn1_id);
    button_init(&btn2, read_button_GPIO, 0, btn2_id);
    button_init(&btn3, read_button_GPIO, 0, btn3_id);
    button_init(&btn4, read_button_GPIO, 0, btn4_id);
    button_init(&btn5, read_button_GPIO, 0, btn5_id);
    button_init(&btn6, read_button_GPIO, 0, btn6_id);
    button_init(&btn7, read_button_GPIO, 0, btn7_id);
    button_init(&btn8, read_button_GPIO, 0, btn8_id);
    button_init(&btn9, read_button_GPIO, 0, btn9_id);

    button_attach(&btn1, PRESS_DOWN,       BTN1_PRESS_DOWN_Handler);
    button_attach(&btn1, PRESS_UP,         BTN1_PRESS_UP_Handler);
    button_attach(&btn2, PRESS_DOWN,       BTN2_PRESS_DOWN_Handler);
    button_attach(&btn2, PRESS_UP,         BTN2_PRESS_UP_Handler);
    button_attach(&btn3, PRESS_DOWN,       BTN3_PRESS_DOWN_Handler);
    button_attach(&btn3, PRESS_UP,         BTN3_PRESS_UP_Handler);
    button_attach(&btn4, PRESS_DOWN,       BTN4_PRESS_DOWN_Handler);
    button_attach(&btn4, PRESS_UP,         BTN4_PRESS_UP_Handler);
    button_attach(&btn5, PRESS_DOWN,       BTN5_PRESS_DOWN_Handler);
    button_attach(&btn5, PRESS_UP,         BTN5_PRESS_UP_Handler);
    button_attach(&btn6, PRESS_DOWN,       BTN6_PRESS_DOWN_Handler);
    button_attach(&btn6, PRESS_UP,         BTN6_PRESS_UP_Handler);
    button_attach(&btn7, PRESS_DOWN,       BTN7_PRESS_DOWN_Handler);
    button_attach(&btn7, PRESS_UP,         BTN7_PRESS_UP_Handler);
    button_attach(&btn8, PRESS_DOWN,       BTN8_PRESS_DOWN_Handler);
    button_attach(&btn8, PRESS_UP,         BTN8_PRESS_UP_Handler);
    button_attach(&btn9, PRESS_DOWN,       BTN9_PRESS_DOWN_Handler);
    button_attach(&btn9, PRESS_UP,         BTN9_PRESS_UP_Handler);
    button_start(&btn1);
    button_start(&btn2);
    button_start(&btn3);
    button_start(&btn4);
    button_start(&btn5);
    button_start(&btn6);
    button_start(&btn7);
    button_start(&btn8);
    button_start(&btn9);
    while(1)
    {
        button_ticks();
        Delay_Ms(5);
    }
}
