/**
  ******************************************************************************
  * @file           : key.h
  * @brief          : key Driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 LDSCITECHE Inc.
  * δ��������ɣ��������������κ���;
  * ��������:2022/1/26
  * �汾��V1.0
  * ��Ȩ���У�����ؾ���
  * Copyright(C) �������ܵ��ӿƼ����޹�˾ LDSCITECHE Inc.
  * All rights reserved		
  *
  ******************************************************************************
  */


#ifndef _KEY_H
#define _KEY_H


#include "ch32v10x.h"

#define     B1_GPIO_PORT        GPIOA
#define     B1_PIN              GPIO_Pin_0
#define     B2_GPIO_PORT        GPIOB
#define     B2_PIN              GPIO_Pin_9
#define     B3_GPIO_PORT        GPIOB
#define     B3_PIN              GPIO_Pin_8
#define     B4_GPIO_PORT        GPIOB
#define     B4_PIN              GPIO_Pin_7
#define     B5_GPIO_PORT        GPIOB
#define     B5_PIN              GPIO_Pin_6
#define     B6_GPIO_PORT        GPIOA
#define     B6_PIN              GPIO_Pin_15
#define     B7_GPIO_PORT        GPIOB
#define     B7_PIN              GPIO_Pin_3
#define     B8_GPIO_PORT        GPIOB
#define     B8_PIN              GPIO_Pin_4
#define     B9_GPIO_PORT        GPIOB
#define     B9_PIN              GPIO_Pin_5


extern void KEY_INIT(void);


#endif



