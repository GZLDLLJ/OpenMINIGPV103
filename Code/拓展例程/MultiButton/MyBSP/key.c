/**
  ******************************************************************************
  * @file           : key.c
  * @brief          : key Driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 LDSCITECHE Inc.
  * 未经作者许可，不得用于其它任何用途
  * 创建日期:2021/11/30
  * 版本：V1.0
  * 版权所有，盗版必究。
  * Copyright(C) 广州联盾电子科技有限公司 LDSCITECHE Inc.
  * All rights reserved		
  *
  ******************************************************************************
  */

#include "key.h"


/*******************************************************************************
* Function Name  : KEY_INIT
* Description    : Initializes GPIOB
* Input          : None
* Return         : None
*******************************************************************************/
void KEY_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6
                                  |GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}













