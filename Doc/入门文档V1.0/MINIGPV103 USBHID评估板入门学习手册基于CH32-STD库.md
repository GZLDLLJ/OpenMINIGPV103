
# 序

由于作者水平有限，文档和视频中难免有出错和讲得不好的地方，欢迎各位读者和观众善意地提出意见和建议，谢谢！



| **实例**             | **描述**                             |
| -------------------- | ------------------------------------ |
| Eg1_Joystick         | 实现一个Joystick摇杆设备             |
| Eg2_WS2812B          | 点亮WS2812B灯珠并实现七彩渐变        |
| Eg3_MultiTimer       | 移植MultiTimer软件定时器模块         |
| Eg4_Mouse            | 实现模拟鼠标功能                     |
| Eg5_KeyBoard         | 实现模拟键盘功能                     |
| Eg6_DoubleJoystick   | 实现一个USB双摇杆                    |
| Eg7_CompositeGMK     | 实现Joystick、MOUSE、Keyboard的组合  |
| Eg8_Gamepad          | 实现游戏手柄Gamepad的功能            |
| Eg9_AbsoluteMouse    | 实现绝对值鼠标的功能                 |
| Eg10_Xinput          | 实现Xbox手柄功能，Xinput（出厂默认） |
| Eg11_Xinput01        | 外接摇杆电位器实现Xbox手柄功能       |
| Eg12_MultiAxisButton | 实现8轴32键摇杆                      |

# 第一部分、硬件概述

## 1.1 实物概图

![在这里插图1.1入图片描述](https://img-blog.csdnimg.cn/dbe1bafc733f41f9acd1dc247c93d09e.png#pic_center)


如图1.1所示Gamepad评估板配置了8个6*6轻触按键，一个摇杆（Joystick），搭载一颗WS2812B灯珠，并将UART1串口，编程接口（SWD），外接Joystick接口，Type-C接口引出;  

## 1.2 Gamepad原理图

Gamepad原理图如图1.2所示，如看不清可打开Doc目录下的PDF文档查阅  
![image](https://img-blog.csdnimg.cn/img_convert/410046a6c84e4ed49e109626ebbf1e13.jpeg)

# 第二部分、软件工具

## 2.1 软件概述

   在 /Software 目录下是常用的工具软件：
1. Dt2_4：配置USB设备Report描述符的工具；
2. USBHID调试助手/呀呀USB： USB调试工具，相当于串口调试助手功能；
3. BUSHound：总线调试工具；
4. USBlyzer：一款专业的USB协议分析软件
5. MounRiver: 编译器；
6. 在线测试工具：https://devicetests.com/  

## 2.2 MounRiver软件入门

MounRiver Studio基于Eclipse GNU版本开发，在保留原平台强大代码编辑功能、便捷组件框架的同时，针对嵌入式C /C++开发，进行了一系列界面、功能、操作方面的修改与优化，以及工具链的指令增添、定制工作。力求打造一款硬件工程师喜爱的、以RISC-V内核为主的嵌入式集成开发环境。大家访问以下链接获取下载：http://mounriver.com/help

# 第三部分、实战训练

## 3.1 实例Eg1_Joystick

本节我们目标是实现Joystick的功能，枚举成XY轴的平面坐标和8个按键的Joystick设备。  

### 3.1.1硬件设计

   ![image](https://img-blog.csdnimg.cn/img_convert/11f3d751e927ddc39bd6ce25ec12df5b.png)
如上图是Joystick原理图，其中VRX1与VRY1是摇杆的电位器输出的电压信号（ADC检测）；SW1则是按键，右侧H1是外接的Joystick口，供接joystick模块使用;  
![image](https://img-blog.csdnimg.cn/img_convert/93b71f719f7c1a841f2461e8f77dab40.png)

如上图是KEY原理图，我们只要配置8个GPIO作为输入去检测按键信号;  

### 3.1.2 软件设计

#### 3.1.2.1 工程树

首先是工程树，我们打开工程，可以看到Project Explorer下Gamepad目录如下图
![image](https://img-blog.csdnimg.cn/img_convert/5dc96768512321ce10a957a7f563edc4.png)
其中

> - __Binaries：__ 二进制文件；
> - __Includes：__ 包含的头文件；
> - __Core：__内核文件，存放core_riscv内核文件；
> - __Debug：__ 存放串口打印和延迟函数相关的文件；
> - __myBSP：__ 我们自己编写的驱动文件；
> - __obj：__ 编译的生成的obj文件；	
> - __Peripheral：__ 这是MCU厂商提供外设相关驱动；
> - __Startup：__ ch32v103的启动文件；
> - __User: __ch32v103的配置文件，中断相关文件，main函数等；

工程目录这里只做一次介绍，后面的样例目录大同小异。

#### 3.1.2.2 系统时钟

我们先打开startup_ch32v10x.S启动文件，我们看到如下代码

```C
  jal  SystemInit
	la t0, main
```

定位到SystemInit

```c
void SystemInit (void)
{
  RCC->CTLR |= (uint32_t)0x00000001;
  RCC->CFGR0 &= (uint32_t)0xF8FF0000;
  RCC->CTLR &= (uint32_t)0xFEF6FFFF;
  RCC->CTLR &= (uint32_t)0xFFFBFFFF;
  RCC->CFGR0 &= (uint32_t)0xFF80FFFF;
  RCC->INTR = 0x009F0000;    
  SetSysClock();
}
```

关于RCC寄存器的配置，请各位自行查阅用户手册；我们接着打开SetSysClock函数

```C
static void SetSysClock(void)
{
#ifdef SYSCLK_FREQ_HSE
  SetSysClockToHSE();
#elif defined SYSCLK_FREQ_24MHz
  SetSysClockTo24();
#elif defined SYSCLK_FREQ_48MHz
  SetSysClockTo48();
#elif defined SYSCLK_FREQ_56MHz
  SetSysClockTo56();  
#elif defined SYSCLK_FREQ_72MHz
  SetSysClockTo72();
#endif
 
 /* If none of the define above is enabled, the HSI is used as System clock
  * source (default after reset) 
	*/ 
}
```

由于我们定义了SYSCLK_FREQ_72MHz，SetSysClockTo72这个函数设置了系统时钟为72M;

#### 3.1.2.3 用户代码

##### 3.1.2.3.1 ADC部分

接下来我们来看看adc部分代码，ADC主要是采集摇杆电位器数据如下：

```c
//初始化ADC
//这里我们仅以规则通道为例
//我们默认将开启通道0~3																	   
void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

	//PA1 作为模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //复位ADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
	
	ADC_StartCalibration(ADC1);	 //开启AD校准
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能

}				  
//获得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		Delay_Ms(5);
	}
	return temp_val/times;
} 	 
```

这段代码是用于配置和读取ADC（模数转换器）的CH32微控制器代码。注释和解释如下：

1. `Adc_Init(void)` 函数用于初始化ADC。它配置了ADC1的工作模式、GPIO引脚和时钟。初始化过程包括校准、使能ADC1等。
2. `Get_Adc(u8 ch)` 函数用于获取指定通道的ADC值。它配置ADC规则组通道，启动ADC转换，等待转换结束，然后返回转换结果。
3. `Get_Adc_Average(u8 ch, u8 times)` 函数用于获取指定通道的ADC值的平均值。它调用 `Get_Adc` 函数多次（由 `times` 参数决定），然后计算这些值的平均值。这有助于减小ADC读数的噪声。

##### 3.1.2.3.2 Button部分

Button主要是独立按键扫描：

```c
/*********************************************************************
 * @fn      buttonGPIOInit
 *
 * @brief   按键IO初始化
 *
 * @param   none
 *
 * @return  none
 */
void buttonGPIOInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能GPIOA时钟

    // 配置GPIOA的Pin 0为输入下拉模式（IPD）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置GPIOA的Pin 15为输入上拉模式（IPU）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 使能GPIOB时钟

    // 配置GPIOB的Pin 3, 4, 5, 6, 7, 8, 9为输入上拉模式（IPU）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6
                                | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 检测按键状态并返回按键值
u8 Button_scan(void)
{
    u8 key = 0;

    // 以下是对各个按键的检测，如果按键按下（电平为低），则设置相应位为1，否则为0
    if (UPKEY() == Bit_RESET)
    {
        key |= BIT0; // 设置key的第0位为1
    }
    else
    {
        key &= (~BIT0); // 设置key的第0位为0
    }

    if (LFKEY() == Bit_RESET)
    {
        key |= BIT1; // 设置key的第1位为1
    }
    else
    {
        key &= (~BIT1); // 设置key的第1位为0
    }

    if (RGKEY() == Bit_RESET)
    {
        key |= BIT2; // 设置key的第2位为1
    }
    else
    {
        key &= (~BIT2); // 设置key的第2位为0
    }

    if (DNKEY() == Bit_RESET)
    {
        key |= BIT3; // 设置key的第3位为1
    }
    else
    {
        key &= (~BIT3); // 设置key的第3位为0
    }

    if (TBKEY() == Bit_RESET)
    {
        key |= BIT4; // 设置key的第4位为1
    }
    else
    {
        key &= (~BIT4); // 设置key的第4位为0
    }

    if (BKKEY() == Bit_RESET)
    {
        key |= BIT5; // 设置key的第5位为1
    }
    else
    {
        key &= (~BIT5); // 设置key的第5位为0
    }

    if (MDKEY() == Bit_RESET)
    {
        key |= BIT6; // 设置key的第6位为1
    }
    else
    {
        key &= (~BIT6); // 设置key的第6位为0
    }

    if (STKEY() == Bit_RESET)
    {
        key |= BIT7; // 设置key的第7位为1
    }
    else
    {
        key &= (~BIT7); // 设置key的第7位为0
    }

    return key; // 返回合并后的按键状态值
}

```

这段代码是用于初始化和检测按键的CH32微控制器代码。注释和解释如下：

1. `buttonGPIOInit(void)` 函数用于初始化按键的GPIO引脚。它配置了不同的GPIO引脚作为输入，一些引脚使用了上拉（IPU）模式，另一些引脚使用了下拉（IPD）模式，这取决于按键硬件连接和工作原理。
2. `Button_scan(void)` 函数用于检测各个按键的状态并返回一个合并后的按键值。它通过调用各个按键的检测函数（如 `UPKEY()`、`LFKEY()` 等）来检测每个按键的状态，如果按键按下（电平为低），则将相应位设置为1，否则为0。然后，将所有按键的状态合并为一个字节，表示按键状态的值。
3. 代码中使用了一些宏定义（如 `Bit_RESET`、`BIT0`、`GPIO_Pin_0` 等），这些宏定义通常用于访问特定的寄存器位或引脚。这些宏定义的具体值和功能可能取决于具体的CH32芯片和库文件版本，因此需要查阅相关文档以了解其含义。

##### 3.1.2.3.3 USB描述符

```c
/********************************** (C) COPYRIGHT *******************************
 * File Name          : composite_km_desc.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/18
 * Description        : All descriptors for the keyboard and mouse composite device.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


/*******************************************************************************/
/* Header File */
#include "usbd_desc.h"

/*******************************************************************************/
/* Device Descriptor */
const uint8_t MyDevDescr[ ] =
{
    0x12,                                                   // bLength
    0x01,                                                   // bDescriptorType
    0x00, 0x02,                                             // bcdUSB
    0x00,                                                   // bDeviceClass
    0x00,                                                   // bDeviceSubClass
    0x00,                                                   // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,                                     // bMaxPacketSize0
    (uint8_t)DEF_USB_VID, (uint8_t)( DEF_USB_VID >> 8 ),    // idVendor
    (uint8_t)DEF_USB_PID, (uint8_t)( DEF_USB_PID >> 8 ),    // idProduct
    0x00, DEF_IC_PRG_VER,                                   // bcdDevice
    0x01,                                                   // iManufacturer
    0x02,                                                   // iProduct
    0x03,                                                   // iSerialNumber
    0x01,                                                   // bNumConfigurations
};

/* Configuration Descriptor Set */
const uint8_t MyCfgDescr[ ] =
{
        0x09, /* bLength: Configuration Descriptor size */
        USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
        USB_CUSTOM_HID_CONFIG_DESC_SIZ,
        /* wTotalLength: Bytes returned */
        0x00,
        0x01,         /*bNumInterfaces: 1 interface*/
        0x01,         /*bConfigurationValue: Configuration value*/
        0x00,         /*iConfiguration: Index of string descriptor describing
        the configuration*/
        0xC0,         /*bmAttributes: bus powered */
        0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

        /************** Descriptor of CUSTOM HID interface ****************/
        /* 09 */
        0x09,         /*bLength: Interface Descriptor size*/
        USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
        0x00,         /*bInterfaceNumber: Number of Interface*/
        0x00,         /*bAlternateSetting: Alternate setting*/
        0x02,         /*bNumEndpoints*/
        0x03,         /*bInterfaceClass: CUSTOM_HID*/
        0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
        0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
        0,            /*iInterface: Index of string descriptor*/
        /******************** Descriptor of CUSTOM_HID *************************/
        /* 18 */
        0x09,         /*bLength: CUSTOM_HID Descriptor size*/
        CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
        0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
        0x01,
        0x00,         /*bCountryCode: Hardware target country*/
        0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
        0x22,         /*bDescriptorType*/
        USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
        0x00,
        /******************** Descriptor of Custom HID endpoints ********************/
        /* 27 */
        0x07,          /*bLength: Endpoint Descriptor size*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

        CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
        0x03,          /*bmAttributes: Interrupt endpoint*/
        CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
        0x00,
        CUSTOM_HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
        /* 34 */

        0x07,          /* bLength: Endpoint Descriptor size */
        USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
        CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
        0x03, /* bmAttributes: Interrupt endpoint */
        CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
        0x00,
        CUSTOM_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */
        /* 41 */
};

/* Keyboard Report Descriptor */
const uint8_t JoystickRepDesc[ ] =
{
        0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
        0x09, 0x04,                    // USAGE (Joystick)
        0xa1, 0x01,                    // COLLECTION (Application)
        0xa1, 0x02,                    //     COLLECTION (Logical)
        0x09, 0x30,                    //     USAGE (X)
        0x09, 0x31,                    //     USAGE (Y)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
        0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
        0x46, 0xff, 0x00,              //     PHYSICAL_MAXIMUM (255)
        0x75, 0x08,                    //     REPORT_SIZE (8)
        0x95, 0x02,                    //     REPORT_COUNT (2)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0x05, 0x09,                    //     USAGE_PAGE (Button)
        0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
        0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
        0x95, 0x08,                    //     REPORT_COUNT (8)
        0x75, 0x01,                    //     REPORT_SIZE (1)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0xc0,                          //     END_COLLECTION
        0xc0,                          // END_COLLECTION                                               // End Collection
};


/* Language Descriptor */
const uint8_t MyLangDescr[ ] =
{
    0x04,
    0x03,
    0x09,
    0x04
};

/* Manufacturer Descriptor */
const uint8_t MyManuInfo[ ] =
{
        0x16,0x03,0x4C,0x00,0x44,0x00,0x53,0x00,0x43,0x00,0x49,0x00,0x54,0x00,0x45,0x00,
        0x43,0x00,0x48,0x00,0x45,0x00
};

/* Product Information */
const uint8_t MyProdInfo[ ]  =
{
        0x16,0x03,0x4C,0x00,0x44,0x00,0x4A,0x00,0x6F,0x00,0x79,0x00,0x73,0x00,0x74,0x00,
        0x69,0x00,0x63,0x00,0x6B,0x00
};

/* Serial Number Information */
u8 USBD_StringSerial[USB_SIZ_STRING_SERIAL] = {
        USB_SIZ_STRING_SERIAL,
        USB_DESC_TYPE_STRING,};

/**
  * @brief  Convert Hex 32Bits value into char
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer
  * @param  len: buffer length
  * @retval None
  */
void IntToUnicode(uint32_t value, uint8_t * pbuf, uint8_t len)
{
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++)
  {
    if (((value >> 28)) < 0xA)
    {
      pbuf[2 * idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}
void Get_SerialNum(void)
{
  uint32_t deviceserial0, deviceserial1, deviceserial2;

  deviceserial0 = *(uint32_t *) DEVICE_ID1;
  deviceserial1 = *(uint32_t *) DEVICE_ID2;
  deviceserial2 = *(uint32_t *) DEVICE_ID3;

  deviceserial0 += deviceserial2;

  if (deviceserial0 != 0)
  {
    IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
    IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
  }
}

```

我们介绍几个Usb描述符和相关函数

1. 设备描述符（Device Descriptor）：
   - `MyDevDescr` 数组包含了设备描述符的定义，它是一种用于标识USB设备的描述符。
   - 设备描述符包括了设备的一些基本信息，如USB协议版本、设备类别、设备子类、制造商ID、产品ID、设备版本等。
2. 配置描述符（Configuration Descriptor）：
   - `MyCfgDescr` 数组包含了配置描述符的定义，它描述了USB设备的配置。
   - 配置描述符包括了配置的一些基本信息，如配置值、电源属性、最大功率等。
   - 这个配置似乎包含了一个自定义的HID接口，用于键盘和鼠标模拟。
3. Joystick报告描述符（JoystickReport Descriptor）：
   - `JoystickRepDesc` 数组包含了一个自定义的HID报告描述符，用于描述Joystick的输入报告格式。
   - 这个描述符指定了输入数据的类型、物理范围、逻辑范围、报告大小等信息，以便主机操作系统正确解释设备的输入。
4. 语言描述符（Language Descriptor）：
   - `MyLangDescr` 数组包含了一个语言描述符，用于描述支持的语言。
   - 这个描述符用于告诉主机操作系统设备支持的语言。
5. 制造商描述符和产品描述符：
   - `MyManuInfo` 和 `MyProdInfo` 数组包含了制造商信息和产品信息的字符串描述符。
   - 这些描述符用于提供制造商名称和产品名称给主机操作系统。
6. 序列号信息：
   - `USBD_StringSerial` 数组似乎是用于存储设备的序列号信息的缓冲区。
7. `IntToUnicode` 函数：
   - 这个函数用于将一个32位的十六进制值转换为对应的Unicode字符形式，并存储到一个缓冲区中。
8. `Get_SerialNum` 函数：

- 这个函数用于获取设备的序列号。
- 它从设备的硬件标识（DEVICE_ID1、DEVICE_ID2、DEVICE_ID3）获取一些数值，并将它们转换为Unicode字符形式，然后存储到 `USBD_StringSerial` 缓冲区中。


### 3.1.3 下载验证

我们把固件程序下载进去可以，打开“设备与打印机”可以看到USB设备枚举成了一个Joystick，如下图。  
![在这里插入图片描述](https://img-blog.csdnimg.cn/4668339ed82f4011b61bbdf40d081715.png#pic_center)


我们可以摇Joystick和按按键可以发现上图游戏控制器界面也跟着响应。

## 3.2实例Eg2_WS2812B

本节我们目标是实现点亮WS2812B。  

### 3.2.1硬件设计

WS2812B-V5是一个集控制电路与发光电路于一体的智能外控LED光源。其外型与一个5050LED灯珠相同，每个元件即为一个像素点。像素点内部包含了智能数字接口数据锁存信号整形放大驱动电路，还包含有高精度的内部振荡器和可编程定电流控制部分，有效保证了像素点光的颜色高度一致。数据协议采用单线归零码的通讯方式，像素点在上电复位以后，DIN端接受从控制器传输过来的数据，首先送过来的24bit数据被第一个像素点提取后，送到像素点内部的数据锁存器，剩余的数据经过内部整形处理电路整形放大后通过DO端口开始转发输出给下一个级联的像素点，每经过一个像素点的传输，信号减少24bit。像素点采用自动整形转发技术，使得该像素点的级联个数不受信号传送的限制，仅受限信号传输速度要求。高达2KHz的端口扫描频率，在高清摄像头的捕捉下都不会出现闪烁现象，非常适合高速移动产品的使用。280μs以上的RESET时间，出现中断也不会引起误复位，可以支持更低频率、价格便宜的MCU。LED具有低电压驱动、环保节能、亮度高、散射角度大、一致性好、超低功率及超长寿命等优点。将控制电路集成于LED上面，电路变得更加简单，体积小，安装更加简便。

#### 3.2.1.1 原理图
![在这里插入图片描述](https://img-blog.csdnimg.cn/f9dc5e1eb77d44a18b81f86d9d25eacc.png#pic_center)


如上图是WS2812B原理图，



#### 3.2.1.2 数据传输时间

![在这里插入图片描述](https://img-blog.csdnimg.cn/013127f8e7524f0c9c7403176acbb75d.png#pic_center)


#### 3.2.1.3 时序波形图

![在这里插入图片描述](https://img-blog.csdnimg.cn/9bbd052725374495ba39420135cac6fa.png#pic_center)


#### 3.2.1.4 数据传输方法

![在这里插入图片描述](https://img-blog.csdnimg.cn/f6b49e387b414376a07754a306863771.png#pic_center)


#### 3.2.1.5  24bit 数据结构

![在这里插入图片描述](https://img-blog.csdnimg.cn/5acfe70ffe0a4ded9780317e911d7dcb.png#pic_center)


### 3.2.2 软件设计

```C
#include <string.h>
#include <ws281x.h>

/* CH1CVR register Definition */
#define TIM3_CH1CVR_ADDRESS    0x40000434

/* Private variables */
u16 send_Buf[NUM]; // 存储要发送给WS281x LED的数据缓冲区

/*********************************************************************
 * @fn      TIM1_PWMOut_Init
 *
 * @brief   初始化TIM1 PWM输出。
 *
 * @param   arr - 周期值。
 *          psc - 分频器值。
 *          ccp - 脉冲值。
 *
 * @return  无
 */
void TIM3_PWMOut_Init(u16 arr, u16 psc, u16 ccp) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    TIM_OCInitTypeDef TIM_OCInitStructure = { 0 };
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = { 0 };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // 配置PA6引脚为复用推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr; // 设置定时器的周期值
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc; // 设置分频器值
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure); // 初始化TIM3定时器

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp; // 设置PWM脉冲值
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // 初始化TIM3通道1为PWM输出

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
}

/*********************************************************************
 * @fn      TIM1_DMA_Init
 *
 * @brief   初始化TIM DMAy通道x的配置。
 *
 * @param   DMA_CHx - DMA通道，范围1到7。
 *          ppadr - 外设基地址。
 *          memadr - 存储器基地址。
 *          bufsize - DMA通道缓冲区大小。
 *
 * @return  无
 */
void TIM3_DMA_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize) {
    DMA_InitTypeDef DMA_InitStructure = { 0 };

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 启用DMA1时钟

    DMA_DeInit(DMA_CHx); // 将DMA通道重置为默认值
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr; // 外设基地址
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr; // 存储器基地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST; // 数据传输方向：从存储器到外设
    DMA_InitStructure.DMA_BufferSize = bufsize; // DMA通道缓冲区大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 外设地址自增禁用
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // 存储器地址自增启用
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 外设数据大小为半字
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; // 存储器数据大小为半字
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; // DMA模式为普通模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; // DMA优先级为中等
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // 存储器到存储器传输禁用
    DMA_Init(DMA_CHx, &DMA_InitStructure); // 初始化DMA通道

    DMA_Cmd(DMA_CHx, ENABLE); // 启用DMA通道
}

/*******************************************************************************
 * Function Name  : WS281xInit
 * Description    : 初始化WS281x LED。
 * Input          : 无
 * Return         : 无
 *******************************************************************************/
void WS281xInit(void) {
    Delay_Ms(50); // 延迟50毫秒
    TIM3_PWMOut_Init(89, 0, 0); // 初始化TIM3用于PWM输出
    TIM3_DMA_Init(DMA1_Channel3, (u32) TIM3_CH1CVR_ADDRESS, (u32) &send_Buf, NUM); // 初始化DMA用于数据传输
    TIM_DMACmd(TIM3, TIM_DMA_Update, ENABLE); // 启用TIM3的DMA请求
    TIM_Cmd(TIM3, ENABLE); // 启用TIM3
    TIM_CtrlPWMOutputs(TIM3, ENABLE); // 启用TIM3的PWM输出
}

void WS281x_SetPixelColor(uint16_t n, uint32_t GRBColor) {
    uint8_t i;
    if (n < PIXEL_NUM) {
        for (i = 0; i < 24; ++i)
            send_Buf[24 * n + i] = (((GRBColor << i) & 0X800000) ? WS1 : WS0);
    }
}

// 更新颜色显示（在设置颜色后将颜色数据存入缓冲区只有执行该函数后才会进行显示）
void ws281x_show(void) {
    DMA_SetCurrDataCounter(DMA1_Channel3, NUM); // 设置DMA通道的当前数据计数器大小
    DMA_Cmd(DMA1_Channel3, ENABLE); // 启用USART1 TX DMA1所指示的通道
    TIM_Cmd(TIM3, ENABLE); // 启用TIM3
    while (DMA_GetFlagStatus(DMA1_FLAG_TC3) == RESET); // 等待DMA传输完成
    TIM_Cmd(TIM3, DISABLE); // 关闭TIM3
    DMA_Cmd(DMA1_Channel3, DISABLE); // 关闭USART1 TX DMA1所指示的通道
    DMA_ClearFlag(DMA1_FLAG_TC3); // 清除DMA传输完成标志
}

uint32_t WS281x_Color(uint8_t red, uint8_t green, uint8_t blue) {
    // 生成32位RGB颜色值
    return green << 16 | red << 8 | blue;
}

// 输入一个0到255的值，获取颜色值。颜色会在红色 - 绿色 - 蓝色 - 红色之间过渡
uint32_t Wheel(uint8_t WheelPos) {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        return WS281x_Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170) {
        WheelPos -= 85;
        return WS281x_Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return WS281x_Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void ws281x_rainbow(void) {
    static uint16_t i, j;

    for (i = 0; i < PIXEL_NUM; i++) {
        WS281x_SetPixelColor(i, Wheel((i + j) & 255));
    }

    ws281x_show();
    if (j++ == 256) {
        j = 0;
    }
}

```

这段代码是用于控制WS281x LED的，它包括初始化LED、设置LED颜色和效果生成的函数。


### 3.2.3 下载验证

我们把固件程序下载进去可以，Joystick同上一实例，板载WS2812实现了七彩渐变效果；

## 3.3实例Eg3_MultiTimer

本节我们目标是学习MultiTimer ；

### 3.3.1MultiTimer

#### 简介
MultiTimer 是一个软件定时器扩展模块，可无限扩展你所需的定时器任务，取代传统的标志位判断方式， 更优雅更便捷地管理程序的时间触发时序。

#### 使用方法
1. 配置系统时间基准接口，安装定时器驱动；

```c
uint64_t PlatformTicksGetFunc(void)
{
    /* Platform implementation */
}

MultiTimerInstall(PlatformTicksGetFunc);
```

2. 实例化一个定时器对象；

```c
MultiTimer timer1;
```

3. 设置定时时间，超时回调处理函数， 用户上下指针，启动定时器；

```c
int MultiTimerStart(&timer1, uint64_t timing, MultiTimerCallback_t callback, void* userData);
```

4. 在主循环调用定时器后台处理函数

```c
int main(int argc, char *argv[])
{
    ...
    while (1) {
        ...
        MultiTimerYield();
    }
}
```

#### 功能限制
1.定时器的时钟频率直接影响定时器的精确度，尽可能采用1ms/5ms/10ms这几个精度较高的tick;

2.定时器的回调函数内不应执行耗时操作，否则可能因占用过长的时间，导致其他定时器无法正常超时；

3.由于定时器的回调函数是在 MultiTimerYield 内执行的，需要注意栈空间的使用不能过大，否则可能会导致栈溢出。

### 3.3.2 软件设计

在这里我们直接上代码：

```C
MultiTimer timer1;
MultiTimer timer2;
MultiTimer timer3;
MultiTimer timer4;

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

// 获取平台时钟计数的函数
uint64_t PlatformTicksGetFunc(void) {
    return uwTick;
}

u8 buttonSemaphore = 0;
u8 button = 0;

// 按钮定时器回调函数
void ButtonTimer1Callback(MultiTimer* timer, void *userData) {
    printf("Button_scan\r\n");

    // 如果按钮信号量为0，扫描按钮状态
    if (buttonSemaphore == 0) {
        button = Button_scan();
        buttonSemaphore = 1;
    }

    // 启动按钮定时器，每5毫秒触发一次，重新调用此回调函数
    MultiTimerStart(timer, 5, ButtonTimer1Callback, userData);
}

u8 adcSemaphore = 0;
u8 xtemp = 0, ytemp = 0;
u16 adxsum = 0, adysum = 0, adcount = 0;

// ADC定时器回调函数
void ADCTimer2Callback(MultiTimer* timer, void *userData) {
    printf("ADC Sample\r\n");

    // 如果ADC信号量为0，执行ADC采样
    if (adcSemaphore == 0) {
        adysum += Get_Adc(1);
        adxsum += Get_Adc(2);
        adcount += 1;

        // 当累积10次采样后，计算平均值并进行处理
        if (++adcount == 10) {
            printf("x=%d, y=%d\r\n", adysum / adcount, adxsum / adcount);
            ytemp = map(adysum / adcount, AD_XMIN, AD_XMAX, 0, 255);
            xtemp = map(adxsum / adcount, AD_YMIN, AD_YMAX, 0, 255);
            adysum = 0;
            adxsum = 0;
            adcount = 0;
            adcSemaphore = 1;
        }
    }

    // 启动ADC定时器，每5毫秒触发一次，重新调用此回调函数
    MultiTimerStart(timer, 5, ADCTimer2Callback, userData);
}

u8 Joystick_Report[3] = { 0 };
u8 LastJoystick_Report[3] = { 0 };

// 摇杆数据定时器回调函数
void JoystickTimer3Callback(MultiTimer* timer, void *userData) {
    printf("Joystick Report\r\n");

    // 如果USB设备已枚举，处理摇杆和按钮数据
    if (USBHD_DevEnumStatus) {
        if (adcSemaphore) {
            adcSemaphore = 0;
            Joystick_Report[1] = ytemp;
            Joystick_Report[0] = xtemp;
        }
        if (buttonSemaphore) {
            buttonSemaphore = 0;
            Joystick_Report[2] = button;
        }

        // 如果摇杆或按钮数据发生变化，发送数据给USB主机
        if (memcmp(LastJoystick_Report, Joystick_Report, sizeof(Joystick_Report) / sizeof(Joystick_Report[0])) != 0) {
            USBHD_Endp_DataUp(DEF_UEP1, Joystick_Report, sizeof(Joystick_Report) / sizeof(Joystick_Report[0]), DEF_UEP_CPY_LOAD);
        }

        // 复制当前的摇杆和按钮数据以供下次比较
        memcpy(LastJoystick_Report, Joystick_Report, sizeof(Joystick_Report) / sizeof(Joystick_Report[0]));
    }

    // 启动摇杆数据定时器，每5毫秒触发一次，重新调用此回调函数
    MultiTimerStart(timer, 5, JoystickTimer3Callback, userData);
}

// WS2812B LED控制定时器回调函数
void WS2812BTimer4Callback(MultiTimer* timer, void *userData) {
    printf("WS2812B\r\n");
    ws281x_rainbow();

    // 启动WS2812B LED控制定时器，每100毫秒触发一次，重新调用此回调函数
    MultiTimerStart(timer, 100, WS2812BTimer4Callback, userData);
}

// 初始化系统轮询功能
void PollSystemInit(void) {
    MultiTimerInstall(PlatformTicksGetFunc);

    // 启动各个定时器，以触发相应的回调函数
    MultiTimerStart(&timer1, 5, ButtonTimer1Callback, NULL);
    MultiTimerStart(&timer2, 5, ADCTimer2Callback, NULL);
    MultiTimerStart(&timer3, 5, JoystickTimer3Callback, NULL);
    MultiTimerStart(&timer4, 100, WS2812BTimer4Callback, NULL);
}

```

这段代码主要是嵌入式系统中用于处理按钮、ADC采样、摇杆数据、和WS2812B LED控制的定时器和回调函数的实现。

通过定时器回调函数将各个任务模块化；

### 3.3.3 下载验证

我们把固件程序下载进去可以，效果同上一实例，板载WS2812实现了七彩渐变效果；

## 3.4实例Eg4_Mouse

本节的目标是实现模拟鼠标的功能；

### 3.4.1硬件说明

摇杆XY模拟鼠标的XY轴，而摇杆自带的按键则作为鼠标中键；

另外板载独立按键SW2、SW3、SW4、SW5分别作为鼠标的滚轮上下和鼠标左右键；

### 3.4.2 软件设计

在这里我们直接替换报表描述符为鼠标的

```C
const uint8_t MouseRepDesc[ ] =
{
        0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
        0x09, 0x02,                    // USAGE (Mouse)
        0xa1, 0x01,                    // COLLECTION (Application)
        0x09, 0x01,                    //   USAGE (Pointer)
        0xa1, 0x00,                    //   COLLECTION (Physical)
        0x05, 0x09,                    //     USAGE_PAGE (Button)
        0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
        0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
        0x95, 0x03,                    //     REPORT_COUNT (3)
        0x75, 0x01,                    //     REPORT_SIZE (1)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0x95, 0x01,                    //     REPORT_COUNT (1)
        0x75, 0x05,                    //     REPORT_SIZE (5)
        0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
        0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
        0x09, 0x30,                    //     USAGE (X)
        0x09, 0x31,                    //     USAGE (Y)
        0x09, 0x38,                    //     USAGE (Wheel)
        0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
        0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
        0x75, 0x08,                    //     REPORT_SIZE (8)
        0x95, 0x03,                    //     REPORT_COUNT (3)
        0x81, 0x06,                    //     INPUT (Data,Var,Rel)
        0xc0,                          //   END_COLLECTION
        0xc0,                          //END_COLLECTION
};

```

这段代码是摇杆数据和按键处理一部分，用于处理来自操纵杆（joystick）的输入数据，并将其通过USB传输。

```c
void JoystickTimer3Callback(MultiTimer* timer, void *userData) {
    printf("Joystick Report\r\n");  // 打印"Joystick Report"消息

    memset(Joystick_Report, 0, 4);  // 用0初始化名为Joystick_Report的数组的前4个字节

    if(USBHD_DevEnumStatus) {
        if(adcSemaphore) {  // 检查adcSemaphore是否为真
            adcSemaphore = 0;  // 将adcSemaphore设置为0，可能是一个标志用于控制ADC数据采集
            if(xtemp > (X_BASE + 20)) {
                Joystick_Report[1] = ((xtemp - X_BASE) >> DIV) + 1;  // 计算并设置X轴的值
            }
            if(xtemp < (X_BASE - 20)) {
                Joystick_Report[1] = (u8)-(((X_BASE - xtemp) >> DIV) + 1);  // 计算并设置X轴的值（可能是负数）
            }
            if(ytemp > (Y_BASE + 20)) {
                Joystick_Report[2] = ((ytemp - Y_BASE) >> DIV) + 1;  // 计算并设置Y轴的值
            }
            if(ytemp < (Y_BASE - 20)) {
                Joystick_Report[2] = (u8)-(((Y_BASE - ytemp) >> DIV) + 1);  // 计算并设置Y轴的值（可能是负数）
            }
        }
        if(buttonSemaphore) {  // 检查buttonSemaphore是否为真
            buttonSemaphore = 0;  // 将buttonSemaphore设置为0，可能是一个标志用于控制按钮状态
            Joystick_Report[0] = button;  // 设置按钮状态
            Joystick_Report[3] = wheel;  // 设置滚轮状态
        }
        // 通过USB传输Joystick_Report数据，其中包含了X轴、Y轴、按钮和滚轮的状态
        USBHD_Endp_DataUp(DEF_UEP1, Joystick_Report, sizeof(Joystick_Report) / sizeof(Joystick_Report[0]), DEF_UEP_CPY_LOAD);
    }

    MultiTimerStart(timer, 5, JoystickTimer3Callback, userData);  // 在5毫秒后再次调用此函数，以实现定时器回调
}

```



### 3.4.3 下载验证

我们把固件程序下载进去可以，效果与普通鼠标基本一致；







## 3.5实例Eg5_Keyboard

本节的目标是实现模拟键盘的功能；

### 3.4.1硬件说明

我们将使用SW1作为左shift键，SW2、SW4、SW5、SW3、SW6、SW9、SW8、SW7分别作为1~8按键。 

### 3.4.2 软件设计

在这里我们直接替换报表描述符为键盘的

```C
const uint8_t KeyBoardRepDesc[ ] =
{
        0x05, 0x01, // USAGE_PAGE (Generic Desktop)
        0x09, 0x06, // USAGE (Keyboard)
        0xa1, 0x01, // COLLECTION (Application)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
        0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0x01, // LOGICAL_MAXIMUM (1)
        0x75, 0x01, // REPORT_SIZE (1)
        0x95, 0x08, // REPORT_COUNT (8)
        0x81, 0x02, // INPUT (Data,Var,Abs)
        0x95, 0x01, // REPORT_COUNT (1)
        0x75, 0x08, // REPORT_SIZE (8)
        0x81, 0x03, // INPUT (Cnst,Var,Abs)
        0x95, 0x05, // REPORT_COUNT (5)
        0x75, 0x01, // REPORT_SIZE (1)
        0x05, 0x08, // USAGE_PAGE (LEDs)
        0x19, 0x01, // USAGE_MINIMUM (Num Lock)
        0x29, 0x05, // USAGE_MAXIMUM (Kana)
        0x91, 0x02, // OUTPUT (Data,Var,Abs)
        0x95, 0x01, // REPORT_COUNT (1)
        0x75, 0x03, // REPORT_SIZE (3)
        0x91, 0x03, // OUTPUT (Cnst,Var,Abs)
        0x95, 0x06, // REPORT_COUNT (6)
        0x75, 0x08, // REPORT_SIZE (8)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0xFF, // LOGICAL_MAXIMUM (255)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
        0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
        0x81, 0x00, // INPUT (Data,Ary,Abs)
        0xC0        // END_COLLECTION
};

```

然后按照报告描述符，我们通过捕获键盘数据可以得到如下协议

键盘发送给PC的数据每次8个字节
BYTE1 BYTE2 BYTE3 BYTE4 BYTE5 BYTE6 BYTE7 BYTE8
定义分别是：
BYTE1 --
    |--bit0:  Left Control是否按下，按下为1 
    |--bit1:  Left Shift 是否按下，按下为1 
    |--bit2:  Left Alt  是否按下，按下为1 
    |--bit3:  Left GUI  是否按下，按下为1 
    |--bit4:  Right Control是否按下，按下为1 
    |--bit5:  Right Shift 是否按下，按下为1 
    |--bit6:  Right Alt  是否按下，按下为1 
    |--bit7:  Right GUI  是否按下，按下为1 
BYTE2 -- 保留字节
BYTE3--BYTE8 -- 这六个为普通按键

根据我们之前定好的SW1作为左shift按键，所以这里BYTE1的bit1位1即为按下，其他是1~8键则依次填充BYTE3~BYTE8；

```c
void Button_Handle(u8* Buf)
{

    uint8_t i=2;
    if(SW1()==Bit_SET)//SHIFT
    {
        Buf[0]|=0x02;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }else{
        Buf[0]&=~0x02;
    }
    if(UPKEY()==Bit_RESET)
    {
        Buf[i]=CODE1;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }
    if(LFKEY()==Bit_RESET)
    {
        Buf[i]=CODE2;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }

    if(RGKEY()==Bit_RESET)
    {
        Buf[i]=CODE3;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }

    if(DNKEY()==Bit_RESET)
    {
        Buf[i]=CODE4;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }
    if((TBKEY())==Bit_RESET)//2@
    {

        Buf[i]=CODE5;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }
    if((BKKEY())==Bit_RESET)//左CTRL
    {

        Buf[i]=CODE6;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }
    if((MDKEY())==Bit_RESET)//左ALT
    {

        Buf[i]=CODE7;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }

    if((STKEY())==Bit_RESET)//I
    {

        Buf[i]=CODE8;
        if(++i==8)//切换到下个位置。
        {
            i=2;
        }
    }


}

```

最后是上报报文

```c
void keyBoardTimer3Callback(MultiTimer* timer, void *userData) {

    u8 keyBoardReport[8] = { 0 };
    static u8 lastKeyboardReport[8] ={0};
    printf("Keyboard Report\r\n");

    if( USBHD_DevEnumStatus )
    {
        Button_Handle(keyBoardReport);

        if(memcmp(keyBoardReport,lastKeyboardReport,sizeof(keyBoardReport) / sizeof(keyBoardReport[0]))!=0)
        {
            USBHD_Endp_DataUp( DEF_UEP1, keyBoardReport,sizeof(keyBoardReport) / sizeof(keyBoardReport[0]), DEF_UEP_CPY_LOAD );
        }

        memcpy(lastKeyboardReport,keyBoardReport,8);
    }
    MultiTimerStart(timer, 5, keyBoardTimer3Callback, userData);
}
```



### 3.4.3 下载验证

我们把固件程序下载进去可以，效果与键盘的shift，12345678基本一致；

## 3.6 实例Eg6_DoubleJoystick

目标是实现一个USB带两个joystick摇杆；功能完全与实例Eg1_Joystick一致；

### 3.6.1硬件设计

参考原理图； 

### 3.6.2 软件设计

首先要修改的是报表描述符：

```c
/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
	0x85, 0x01,                    //   REPORT_ID (1)
    0xa1, 0x02,                    //     COLLECTION (Logical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
    0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
    0x46, 0xff, 0x00,              //     PHYSICAL_MAXIMUM (255)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0, 0xc0,                    //               END_COLLECTION

    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x04,                    // USAGE (Joystick)
    0xa1, 0x01,                    // COLLECTION (Application)
	0x85, 0x02,                    //   REPORT_ID (2)
    0xa1, 0x02,                    //     COLLECTION (Logical)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
    0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
    0x46, 0xff, 0x00,              //     PHYSICAL_MAXIMUM (255)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x02,                    //     REPORT_COUNT (2)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x95, 0x08,                    //     REPORT_COUNT (8)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0xc0,                      //               END_COLLECTION
  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};
```

与前面几个实例不同的是，这里增加了Report ID，就是说每个USB接口都支持多个Report ID;每个Report ID都支持不同的报表描述符，如某些复合的USB键鼠一体设备，就是通过USB Report ID区分的键盘与鼠标的；

数据解析：XY_Handle是解析X，Y坐标的，key_scan是对8颗按键进行扫描，Joystick_Report[0]就是Report ID，占用1Byte，也就是如果带宽允许，最大支持255个报表；

```c
void GamepadHandle(void)
{
	XY_Handle();
	Joystick_Report[0]=1;//Report 1;
	Joystick_Report[1]=Y;
	Joystick_Report[2]=X;
	key_scan(&Joystick_Report[3]);	
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(u8*)&Joystick_Report, JOYBUFSIZE);
	HAL_Delay(8);
	Joystick_Report[0]=2;//Report 1;
	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS,(u8*)&Joystick_Report, JOYBUFSIZE);
	HAL_Delay(8);	
}
```

### 3.6.3 下载验证

我们把固件程序下载进去，可以看到游戏控制器界面有两个控制器，调开属性界面两个都可以控制；
![image-20231021233605937](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20231021233605937.png)

我们可以打开Bus Hound，抓取报文，端点0传输的是枚举过程，然后我们看Device：66.1的4个字节的，报文01 7e 7d 00是Report id为1的报文， 报文02 7e 7d 00是Report id为2的报文，因为我们上报的是相同的数据，就ID不同，故而控制的是两个joystick设备；
![image](https://img-blog.csdnimg.cn/img_convert/45b44e665855b6011ae9e96de4868c7c.png)





## 3.7 实例Eg7_ComDev_JMK

本节目标是实现Joystick、Mouse和Keyboard的组合，即把实例Eg1_Joystick、实例Eg4_Mouse与实例Eg5_KeyBoard组合成一个设备，通过按键SW2按下依次切换成Joystick、Mouse和Keyboard设备，而2812灯珠则通过红绿蓝指示切换三种不同的设备。

### 3.7.1硬件设计

参考原理图； 

### 3.7.2 软件设计

准备Joystick、Mouse和Keyboard三个报表，这三个报表分别是实例Eg1_Joystick、实例Eg4_Mouse与实例Eg5_KeyBoard的报表，

```c
/* Joystick Report Descriptor */
const uint8_t joystickRepDesc[ ] =
{
        0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
        0x09, 0x04,                    // USAGE (Joystick)
        0xa1, 0x01,                    // COLLECTION (Application)
        0xa1, 0x02,                    //     COLLECTION (Logical)
        0x09, 0x30,                    //     USAGE (X)
        0x09, 0x31,                    //     USAGE (Y)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x00,              //     LOGICAL_MAXIMUM (255)
        0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
        0x46, 0xff, 0x00,              //     PHYSICAL_MAXIMUM (255)
        0x75, 0x08,                    //     REPORT_SIZE (8)
        0x95, 0x02,                    //     REPORT_COUNT (2)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0x05, 0x09,                    //     USAGE_PAGE (Button)
        0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
        0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
        0x95, 0x08,                    //     REPORT_COUNT (8)
        0x75, 0x01,                    //     REPORT_SIZE (1)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0xc0,                          //     END_COLLECTION
        0xc0                           // END_COLLECTION
};
/* Mouse Report Descriptor */
const uint8_t mouseReportDesc[ ] =
{
        0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
        0x09, 0x02,                    // USAGE (Mouse)
        0xa1, 0x01,                    // COLLECTION (Application)
        0x09, 0x01,                    //   USAGE (Pointer)
        0xa1, 0x00,                    //   COLLECTION (Physical)
        0x05, 0x09,                    //     USAGE_PAGE (Button)
        0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
        0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
        0x95, 0x03,                    //     REPORT_COUNT (3)
        0x75, 0x01,                    //     REPORT_SIZE (1)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0x95, 0x01,                    //     REPORT_COUNT (1)
        0x75, 0x05,                    //     REPORT_SIZE (5)
        0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
        0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
        0x09, 0x30,                    //     USAGE (X)
        0x09, 0x31,                    //     USAGE (Y)
        0x09, 0x38,                    //     USAGE (Wheel)
        0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
        0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
        0x75, 0x08,                    //     REPORT_SIZE (8)
        0x95, 0x03,                    //     REPORT_COUNT (3)
        0x81, 0x06,                    //     INPUT (Data,Var,Rel)
        0xc0,                          //   END_COLLECTION
        0xc0                           // END_COLLECTION
};
/* Keyboard Report Descriptor */
const uint8_t keyBoardReportDesc[ ] =
{
        0x05, 0x01, // USAGE_PAGE (Generic Desktop)
        0x09, 0x06, // USAGE (Keyboard)
        0xa1, 0x01, // COLLECTION (Application)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0xe0, // USAGE_MINIMUM (Keyboard LeftControl)
        0x29, 0xe7, // USAGE_MAXIMUM (Keyboard Right GUI)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0x01, // LOGICAL_MAXIMUM (1)
        0x75, 0x01, // REPORT_SIZE (1)
        0x95, 0x08, // REPORT_COUNT (8)
        0x81, 0x02, // INPUT (Data,Var,Abs)
        0x95, 0x01, // REPORT_COUNT (1)
        0x75, 0x08, // REPORT_SIZE (8)
        0x81, 0x03, // INPUT (Cnst,Var,Abs)
        0x95, 0x05, // REPORT_COUNT (5)
        0x75, 0x01, // REPORT_SIZE (1)
        0x05, 0x08, // USAGE_PAGE (LEDs)
        0x19, 0x01, // USAGE_MINIMUM (Num Lock)
        0x29, 0x05, // USAGE_MAXIMUM (Kana)
        0x91, 0x02, // OUTPUT (Data,Var,Abs)
        0x95, 0x01, // REPORT_COUNT (1)
        0x75, 0x03, // REPORT_SIZE (3)
        0x91, 0x03, // OUTPUT (Cnst,Var,Abs)
        0x95, 0x06, // REPORT_COUNT (6)
        0x75, 0x08, // REPORT_SIZE (8)
        0x15, 0x00, // LOGICAL_MINIMUM (0)
        0x25, 0xFF, // LOGICAL_MAXIMUM (255)
        0x05, 0x07, // USAGE_PAGE (Keyboard)
        0x19, 0x00, // USAGE_MINIMUM (Reserved (no event indicated))
        0x29, 0x65, // USAGE_MAXIMUM (Keyboard Application)
        0x81, 0x00, // INPUT (Data,Ary,Abs)

        0xC0    /*     END_COLLECTION              */
};
```

在USBHD_Device_Endp_Init需要使能配置端点2,3；

```c
void USBHD_Device_Endp_Init( void )
{
    /* Initiate endpoint mode settings, please modify them according to your project. */
    R8_UEP4_1_MOD = RB_UEP1_TX_EN;
    R8_UEP2_3_MOD = RB_UEP2_TX_EN;

    /* Initiate DMA address, please modify them according to your project. */
    R16_UEP0_DMA = (uint16_t)(uint32_t)USBHD_EP0_Buf;
    R16_UEP1_DMA = (uint16_t)(uint32_t)USBHD_EP1_Buf;
    R16_UEP2_DMA = (uint16_t)(uint32_t)USBHD_EP2_Buf;

    /* End-points initial states */
    R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
    R8_UEP1_CTRL = UEP_T_RES_NAK;
    R8_UEP2_CTRL = UEP_T_RES_NAK;

    USBHD_Endp_Busy[ DEF_UEP1] = 0;
}
```



### 3.7.3 下载验证

我们把固件程序下载进去，摇动摇杆，按住SW2大于4s，可依次切换成鼠标模式成摇杆、鼠标、键盘模式，2812显示对应的红绿蓝色。





## 3.8 实例Eg8_Gamepad

本节目标是实现Gamepad，Gamepad和Joystick是有很大区别的，我们先看一下Joystick，如下图Joystick是带有XY轴并带有若干个按键的HID。


![image](https://img-blog.csdnimg.cn/img_convert/31657996ccf1270a25ed07d7b83bc7e3.png)


而Gamepad除了XY轴和按键，还有Z轴，Rx和Ry（旋转），并带有视觉头盔。

![image](https://img-blog.csdnimg.cn/img_convert/c9dac232021b22b109666388d4c34180.png)


本节的就是实现如上图这样一个Gamepad。

### 3.8.1硬件设计

参考原理图； 

### 3.8.2 软件设计

1. 准备Gamepad报表，根据报表定义，buf[0]和buf[1]分别是XY轴，buf[2]和buf[3]分别是Rx和Ry，而buf[4]代表着Z轴，而按钮是有10个所以占用了1~10bit；而视觉头盔是HatSwitch，4bit；另有2bit是无定义的，而报表还有2BYTE是保留字节。这样算下来一共定义了9个byte。

```c
        0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
        0x09, 0x05,                    // USAGE (Game Pad)
        0xa1, 0x01,                    // COLLECTION (Application)
        0xa1, 0x00,                    //   COLLECTION (Physical)
        0x09, 0x30,                    //     USAGE (X)
        0x09, 0x31,                    //     USAGE (Y)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x00,                    //     LOGICAL_MAXIMUM (255)
        0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
        0x46, 0xff, 0x00,                    //     PHYSICAL_MAXIMUM (255)
        0x95, 0x02,                    //     REPORT_COUNT (2)
        0x75, 0x08,                    //     REPORT_SIZE (8)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0xc0,                          //     END_COLLECTION
        0xa1, 0x00,                    //   COLLECTION (Physical)
        0x09, 0x33,                    //     USAGE (Rx)
        0x09, 0x34,                    //     USAGE (Ry)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x00,                    //     LOGICAL_MAXIMUM (255)
        0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
        0x46, 0xff, 0x00,                    //     PHYSICAL_MAXIMUM (255)
        0x95, 0x02,                    //     REPORT_COUNT (2)
        0x75, 0x08,                    //     REPORT_SIZE (8)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0xc0,                          // END_COLLECTION
        0xa1, 0x00,                    //   COLLECTION (Physical)
        0x09, 0x32,                    //     USAGE (Z)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x26, 0xff, 0x00,                    //     LOGICAL_MAXIMUM (255)
        0x35, 0x00,                    //     PHYSICAL_MINIMUM (0)
        0x46, 0xff, 0x00,                    //     PHYSICAL_MAXIMUM (255)
        0x95, 0x01,                    //     REPORT_COUNT (1)
        0x75, 0x08,                    //     REPORT_SIZE (8)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0xc0,                          // END_COLLECTION
        0x05, 0x09,                    //   USAGE_PAGE (Button)
        0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
        0x29, 0x0a,                    //   USAGE_MAXIMUM (Button 10)
        0x95, 0x0a,                    //   REPORT_COUNT (10)
        0x75, 0x01,                    //   REPORT_SIZE (1)
        0x81, 0x02,                    //   INPUT (Data,Var,Abs)
        0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
        0x09, 0x39,                    //   USAGE (Hat switch)
        0x15, 0x01,                    //   LOGICAL_MINIMUM (1)
        0x25, 0x08,                    //   LOGICAL_MAXIMUM (8)
        0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
        0x46, 0x3b, 0x10,              //   PHYSICAL_MAXIMUM (4155)
        0x66, 0x0e, 0x00,                    //   UNIT (None)
        0x75, 0x04,                    //   REPORT_SIZE (4)
        0x95, 0x01,                    //   REPORT_COUNT (1)
        0x81, 0x42,                    //   INPUT (Data,Var,Abs,Null)
        0x75, 0x02,                    //   REPORT_SIZE (2)
        0x95, 0x01,                    //   REPORT_COUNT (1)
        0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
        0x75, 0x08,                    //   REPORT_SIZE (8)
        0x95, 0x02,                    //   REPORT_COUNT (2)
        0x81, 0x03,                     //   INPUT (Cnst,Var,Abs)

      0xC0    /*     END_COLLECTION              */    
```

2. 我们只需要修改一下报表描述符即可，然后，我们需要对硬件摇杆和按键进行映射，主要映射XY轴，以及视觉头盔，并且映射button1~4。如下：

```c
void buttonHandler(u8* Hat,u8* But)
{
    //Hat
    if((UPKEY()==0)&&(DNKEY()==0)&&(LFKEY()==0)&&(RGKEY()!=0)) //0001
    {
        Hat[0]|=HATSW7;
    }else if((UPKEY()==0)&&(DNKEY()==0)&&(LFKEY()!=0)&&(RGKEY()==0))//0010
    {
        Hat[0]|=HATSW3;
    }else if((UPKEY()==0)&&(DNKEY()!=0)&&(LFKEY()==0)&&(RGKEY()==0))//0100
    {
        Hat[0]|=HATSW1;
    }else if((UPKEY()==0)&&(DNKEY()!=0)&&(LFKEY()==0)&&(RGKEY()!=0))//0101
    {
        Hat[0]|=HATSW8;
    }else if((UPKEY()==0)&&(DNKEY()!=0)&&(LFKEY()!=0)&&(RGKEY()==0))//0110
    {
        Hat[0]|=HATSW2;
    }else if((UPKEY()==0)&&(DNKEY()!=0)&&(LFKEY()!=0)&&(RGKEY()!=0))//0111
    {
        Hat[0]|=HATSW1;
    }else if((UPKEY()!=0)&&(DNKEY()==0)&&(LFKEY()==0)&&(RGKEY()==0))//1000
    {
        Hat[0]|=HATSW5;
    }else if((UPKEY()!=0)&&(DNKEY()==0)&&(LFKEY()==0)&&(RGKEY()!=0))//1001
    {
        Hat[0]|=HATSW6;
    }else if((UPKEY()!=0)&&(DNKEY()==0)&&(LFKEY()!=0)&&(RGKEY()==0))//1010
    {
        Hat[0]|=HATSW4;
    }else if((UPKEY()!=0)&&(DNKEY()==0)&&(LFKEY()!=0)&&(RGKEY()!=0))//1011
    {
        Hat[0]|=HATSW5;
    }else if((UPKEY()!=0)&&(DNKEY()!=0)&&(LFKEY()==0)&&(RGKEY()!=0))//1101
    {
        Hat[0]|=HATSW7;
    }else if((UPKEY()!=0)&&(DNKEY()!=0)&&(LFKEY()!=0)&&(RGKEY()==0))//1110
    {
        Hat[0]|=HATSW3;
    }else{
        Hat[0]&=(~0x3C);
    }
    //Button
     if(STKEY()==0)
    {
        But[0]|=BIT0;
    }else{
        But[0]&=(~BIT0);
    }
    if(MDKEY()==0)
    {
        But[0]|=BIT1;
    }else{
        But[0]&=(~BIT1);
    }
    if(BKKEY()==0)
    {
        But[0]|=BIT2;
    }else{
        But[0]&=(~BIT2);
    }
    if(TBKEY()==0)
    {
        But[0]|=BIT3;
    }else{
        But[0]&=(~BIT3);
    }
}

void ADCTimer2Callback(MultiTimer* timer, void *userData) {
    printf("ADC Sample\r\n");

    if (adcSemaphore == 0) {

        adysum+=Get_Adc(1);
        adxsum+=Get_Adc(2);

        if(++adcount==10)
        {
            printf("x=%d,y=%d\r\n",adysum/adcount,adxsum/adcount);
            ytemp = map(adysum/adcount, AD_XMIN, AD_XMAX, 0, 255);
            xtemp = map(adxsum/adcount, AD_YMIN, AD_YMAX, 0, 255);
            adysum=0;
            adxsum=0;
            adcount=0;
            adcSemaphore = 1;
        }


    }
    MultiTimerStart(timer, 5, ADCTimer2Callback, userData);
}
```

### 3.8.3 下载验证

我们把固件程序下载进去，界面如下图，摇动摇杆，XY轴响应，按下左边4个按键可控制视觉头盔。板子中间四颗按键实现1~4按钮按下。

![image](https://img-blog.csdnimg.cn/img_convert/f0c50ac4a369c06ef168db7a7ce9b632.png)



## 3.9 实例Eg9_AbsoluteMouse

本节目标是实现绝对值鼠标，绝对值鼠标和相对鼠标是有很大区别的，下面就让我们一起探讨如何实现一个绝对值鼠标.

### 3.9.1硬件设计

参考原理图； 

### 3.9.2 软件设计

1. 准备绝对值鼠标报表，

```c
const uint8_t JoystickRepDesc[ ] =
{
        0x05, 0x01,       // USAGE_PAGE (Generic Desktop)
        0x09, 0x02,       // USAGE (Mouse)
        0xa1, 0x01,       // COLLECTION (Application)
        0x09, 0x01,       //   USAGE (Pointer)
        0xa1, 0x00,       //   COLLECTION (Physical)
        0x05, 0x09,       //     USAGE_PAGE (Button)
        0x19, 0x01,       //     USAGE_MINIMUM (Button 1)
        0x29, 0x03,       //     USAGE_MAXIMUM (Button 3)
        0x95, 0x03,       //     REPORT_COUNT (3)
        0x75, 0x01,       //     REPORT_SIZE (1)
        0x15, 0x00,       //     LOGICAL_MINIMUM (0)
        0x25, 0x01,       //     LOGICAL_MAXIMUM (1)
        0x81, 0x02,       //     INPUT (Data,Var,Abs)
        0x95, 0x01,       //     REPORT_COUNT (1)
        0x75, 0x05,       //     REPORT_SIZE (5)
        0x81, 0x01,       //     INPUT (Cnst,Ary,Abs)

        0x05, 0x01,       //     USAGE_PAGE (Generic Desktop)
        0x09, 0x38,       //     USAGE (Wheel)
        0x15, 0x81,       //     LOGICAL_MINIMUM (-127)
        0x25, 0x7f,       //     LOGICAL_MAXIMUM (127)
        0x95, 0x01,       //     REPORT_COUNT (1)
        0x75, 0x08,       //     REPORT_SIZE (8)
        0x81, 0x06,       //     INPUT (Cnst,Ary,Abs)
        0x16, 0x00,0x00,    //     LOGICAL_MINIMUM (0)
        0x26, 0x00,0x01,  //     LOGICAL_MAXIMUM (256)
        0x36, 0x00,0x00,    //     PHYSICAL_MINIMUM (0)
        0x46, 0x00,0x01,    //     PHYSICAL_MAXIMUM (256)
        0x66, 0x00,0x00, //     UNIT (None)
        0x09, 0x30,       //     USAGE (X)
        0x09, 0x31,       //     USAGE (Y)
        0x75, 0x10,       //     REPORT_SIZE (16)
        0x95, 0x02,       //     REPORT_COUNT (2)
        0x81, 0x62,       //     INPUT (Data,Var,Abs,NPrf,Null)
        0xc0,             //     END_COLLECTION
        0xc0              // END_COLLECTION                                            // End Collection
};
```

2. 根据报表定义，可以得到如下定义，buttons的bit0、bit1、bit2分别对应左键、右键、中键，而wheel代表着鼠标滚轮，最后x与y是定义XY轴的。

```c
typedef struct mouseHID_t
{
    uint8_t buttons;
    int8_t wheel;
    uint16_t x;
    uint16_t y;
} mouseHID_T;
```

3. 最后解析数据。

```c
void buttonHandler(int8_t* whl,u8* But)
{
    static uint16_t c_tick=0;
    //Button
    if(LFKEY()==0)
    {
        But[0]|=BIT0;
    } else {
        But[0]&=(~BIT0);
    }
    if(RGKEY()==0)
    {
        But[0]|=BIT1;
    } else {
        But[0]&=(~BIT1);
    }
    if(SW1()==0)
    {
        But[0]&=(~BIT2);
    } else {
        But[0]|=BIT2;
    }

    if(UPKEY()==0)
    {
        if(c_tick++>5)
        {
            whl[0]=1;
            c_tick=0;
        }
    }
    if(DNKEY()==0)
    {
        if(c_tick++>5)
        {
            whl[0]=-1;
            c_tick=0;
        }
    }
}
void ButtonTimer1Callback(MultiTimer* timer, void *userData) {
    printf("Button_scan\r\n");
    if (buttonSemaphore == 0) {
        button=0;
        whl=0;
        buttonHandler(&whl,&button);
        buttonSemaphore = 1;
    }

    MultiTimerStart(timer, 5, ButtonTimer1Callback, userData);
}
u8 adcSemaphore = 0;
u8 xtemp = 0, ytemp = 0;
u16 adxsum=0,adysum=0,adcount=0;
void ADCTimer2Callback(MultiTimer* timer, void *userData) {
    printf("ADC Sample\r\n");

    if (adcSemaphore == 0) {

        adysum+=Get_Adc(1);
        adxsum+=Get_Adc(2);

        if(++adcount==10)
        {
            printf("x=%d,y=%d\r\n",adysum/adcount,adxsum/adcount);
            ytemp = map(adysum/adcount, AD_XMIN, AD_XMAX, 0, 255);
            xtemp = map(adxsum/adcount, AD_YMIN, AD_YMAX, 0, 255);
            adysum=0;
            adxsum=0;
            adcount=0;
            adcSemaphore = 1;
        }


    }
    MultiTimerStart(timer, 5, ADCTimer2Callback, userData);
}

mouseHID_T MouseBuf;
mouseHID_T lastMouse_Buffer;
void JoystickTimer3Callback(MultiTimer* timer, void *userData) {

    printf("Mouse Report\r\n");
    if( USBHD_DevEnumStatus )
    {
        memset(&MouseBuf, 0,sizeof(MouseBuf));
        if(adcSemaphore)
        {
            adcSemaphore=0;
            MouseBuf.x=xtemp;
            MouseBuf.y=ytemp;
        }
        if(buttonSemaphore)
        {
            buttonSemaphore=0;
            MouseBuf.buttons=button;
            MouseBuf.wheel=whl;
        }

        if(lastMouse_Buffer.x!=MouseBuf.x||lastMouse_Buffer.y!=MouseBuf.y||
                lastMouse_Buffer.buttons!=MouseBuf.buttons||MouseBuf.wheel!=0)
        {
            USBHD_Endp_DataUp( DEF_UEP1, (u8*)&MouseBuf,sizeof(MouseBuf), DEF_UEP_CPY_LOAD );
        }
        lastMouse_Buffer.x=MouseBuf.x;
        lastMouse_Buffer.y=MouseBuf.y;
        lastMouse_Buffer.buttons=MouseBuf.buttons;
        lastMouse_Buffer.wheel=MouseBuf.wheel;

    }
    MultiTimerStart(timer, 5, JoystickTimer3Callback, userData);
}
```



### 3.9.3 下载验证

我们把固件程序下载进去，鼠标居中，按键滚轮基本同相对鼠标，而XY轴则是整个屏幕是作为XY轴的。







## 3.10 实例Eg10_Xinput

本节目标是实现实现Xbox 360 Controller for Windows.

### 3.10.1硬件设计

参考原理图； 

### 3.10.2 软件设计

1. **修改设备描述符**

```c
#define DEF_USB_VID                   0x045e
#define DEF_USB_PID                   0x028e

const uint8_t MyDevDescr[ ] =
{
    0x12,                                                   // bLength
    0x01,                                                   // bDescriptorType
    0x00, 0x02,                                             // bcdUSB
    0x00,                                                   // bDeviceClass
    0x00,                                                   // bDeviceSubClass
    0x00,                                                   // bDeviceProtocol
    DEF_USBD_UEP0_SIZE,                                     // bMaxPacketSize0
    (uint8_t)DEF_USB_VID, (uint8_t)( DEF_USB_VID >> 8 ),    // idVendor
    (uint8_t)DEF_USB_PID, (uint8_t)( DEF_USB_PID >> 8 ),    // idProduct
    0x00, DEF_IC_PRG_VER,                                   // bcdDevice
    0x01,                                                   // iManufacturer
    0x02,                                                   // iProduct
    0x03,                                                   // iSerialNumber
    0x01,                                                   // bNumConfigurations
};
```

**2.修改配置/接口/HID/端点/厂商描述符：**

```c
/* Configuration Descriptor Set */
const uint8_t MyCfgDescr[ ] =
{
        /************** Configuration Descriptor 1 Bus Powered, 500 mA ****************/
        0x09, /* bLength: Configuration Descriptor size */
        USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
        USB_CUSTOM_HID_CONFIG_DESC_SIZ,
        /* wTotalLength: Bytes returned */
        0x00,
        0x04,         /*bNumInterfaces: 1 interface*/
        0x01,         /*bConfigurationValue: Configuration value*/
        0x00,         /*iConfiguration: Index of string descriptor describing
      the configuration*/
        0xA0,         /*bmAttributes: Bus Powered, Remote Wakeup*/
        0xFA,         /*MaxPower 500 mA: this current is used for detecting Vbus*/

        /**************Interface Descriptor 0/0 Vendor-Specific, 2 Endpoints****************/
        /* 09 */
        0x09,         /*bLength: Interface Descriptor size*/
        USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
        0x00,         /*bInterfaceNumber: Number of Interface*/
        0x00,         /*bAlternateSetting: Alternate setting*/
        0x02,         /*bNumEndpoints*/
        0xFF,         /*Vendor-Specific*/
        0x5D,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
        0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
        0x00,            /*iInterface: Index of string descriptor*/
        /**************Unrecognized Class-Specific Descriptor***********************/
        /* 18 */
        0x11,         /*bLength: CUSTOM_HID Descriptor size*/
        0x21, /*bDescriptorType: CUSTOM_HID*/
        0x10,0x01,0x01,0x25,0x81,0x14,0x03,0x03,
        0x03,0x04,0x13,0x02,0x08,0x03,0x03,
        /**************Endpoint Descriptor 81 1 In, Interrupt, 4 ms******************/
        /* 27 */
        0x07,          /*bLength: Endpoint Descriptor size*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

        CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
        0x03,          /*bmAttributes: Interrupt endpoint*/
        CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
        0x00,
        CUSTOM_HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
        /**************Endpoint Descriptor 02 2 Out, Interrupt, 8 ms******************/
        /* 34 */
        0x07,          /*bLength: Endpoint Descriptor size*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

        0x02,     /*bEndpointAddress: Endpoint Address (IN)*/
        0x03,          /*bmAttributes: Interrupt endpoint*/
        0x20, /*wMaxPacketSize: 2 Byte max */
        0x00,
        0x08,          /*bInterval: Polling Interval */

        /**************Interface Descriptor 1/0 Vendor-Specific, 2 Endpoints****************/
        /* 09 */
        0x09,         /*bLength: Interface Descriptor size*/
        USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
        0x01,         /*bInterfaceNumber: Number of Interface*/
        0x00,         /*bAlternateSetting: Alternate setting*/
        0x02,         /*bNumEndpoints*/
        0xFF,         /*Vendor-Specific*/
        0x5D,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
        0x01,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
        0x00,            /*iInterface: Index of string descriptor*/
        /**************Unrecognized Class-Specific Descriptor***********************/
        /* 18 */
        0x1B,         /*bLength: CUSTOM_HID Descriptor size*/
        0x21, /*bDescriptorType: CUSTOM_HID*/
        0x00,0x01,0x01,0x01,0x83,0x40,0x01,0x04,
        0x20,0x16,0x85,0x00,0x00,0x00,0x00,0x00,
        0x00,0x16,0x05,0x00,0x00,0x00,0x00,0x00,
        0x00,
        /**************Endpoint Descriptor 81 1 In, Interrupt, 4 ms******************/
        /* 27 */
        0x07,          /*bLength: Endpoint Descriptor size*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

        0x83,     /*bEndpointAddress: Endpoint Address (IN)*/
        0x03,          /*bmAttributes: Interrupt endpoint*/
        0x20, /*wMaxPacketSize: 2 Byte max */
        0x00,
        0x02,          /*bInterval: Polling Interval */
        /**************Endpoint Descriptor 02 2 Out, Interrupt, 8 ms******************/
        /* 34 */
        0x07,          /*bLength: Endpoint Descriptor size*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

        0x04,     /*bEndpointAddress: Endpoint Address (IN)*/
        0x03,          /*bmAttributes: Interrupt endpoint*/
        0x20, /*wMaxPacketSize: 2 Byte max */
        0x00,
        0x04,          /*bInterval: Polling Interval */

        /**************Interface Descriptor 2/0 Vendor-Specific, 2 Endpoints****************/
        /* 09 */
        0x09,         /*bLength: Interface Descriptor size*/
        USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
        0x02,         /*bInterfaceNumber: Number of Interface*/
        0x00,         /*bAlternateSetting: Alternate setting*/
        0x01,         /*bNumEndpoints*/
        0xFF,         /*Vendor-Specific*/
        0x5D,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
        0x02,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
        0x00,            /*iInterface: Index of string descriptor*/
        /**************Unrecognized Class-Specific Descriptor***********************/
        /* 18 */
        0x09,         /*bLength: CUSTOM_HID Descriptor size*/
        0x21, /*bDescriptorType: CUSTOM_HID*/
        0x00,0x01,0x01,0x22,0x86,0x07,0x00,
        /**************Endpoint Descriptor 81 1 In, Interrupt, 4 ms******************/
        /* 27 */
        0x07,          /*bLength: Endpoint Descriptor size*/
        USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

        0x86,     /*bEndpointAddress: Endpoint Address (IN)*/
        0x03,          /*bmAttributes: Interrupt endpoint*/
        0x20, /*wMaxPacketSize: 2 Byte max */
        0x00,
        0x10,          /*bInterval: Polling Interval */

        /**************nterface Descriptor 3/0 Vendor-Specific, 0 Endpoints****************/
        /* 09 */
        0x09,         /*bLength: Interface Descriptor size*/
        USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
        0x03,         /*bInterfaceNumber: Number of Interface*/
        0x00,         /*bAlternateSetting: Alternate setting*/
        0x00,         /*bNumEndpoints*/
        0xFF,         /*Vendor-Specific*/
        0xFD,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
        0x13,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
        0x04,            /*iInterface: Index of string descriptor*/
        /**************Unrecognized Class-Specific Descriptor***********************/
        /* 18 */
        0x06,         /*bLength: CUSTOM_HID Descriptor size*/
        0x41, /*bDescriptorType: CUSTOM_HID*/
        0x00,0x01,0x01,0x03
};

```

**3. 解析摇杆电位器和按键数据。**

```c
void buttonHandler(u8* packet1,u8* packet2)
{

    //Button
    if(UPKEY()==0)//Y
    {
        packet2[0] |= Y_MASK_ON;
    }else{
        packet2[0] &= Y_MASK_OFF;
    }
    if(DNKEY()==0)//A
    {
        packet2[0] |= A_MASK_ON;
    }else{
        packet2[0] &= A_MASK_OFF;
    }

    if(LFKEY()==0)
    {
        packet2[0] |= X_MASK_ON;
    } else {
        packet2[0] &= X_MASK_OFF;
    }
    if(RGKEY()==0)
    {
        packet2[0] |= B_MASK_ON;
    } else {
        packet2[0] &= B_MASK_OFF;
    }

    if(BKKEY()==0)//BUTTON_BACK
    {
        packet1[0] |= BACK_MASK_ON;
    } else {
        packet1[0] &= BACK_MASK_OFF;
    }
    if(MDKEY()==0)//BUTTON_LB
    {
        packet2[0] |= LB_MASK_ON;
    } else {
        packet2[0] &= LB_MASK_OFF;
    }
    if(STKEY()==0)//BUTTON_START
    {
        packet1[0] |= START_MASK_ON;
    } else {
        packet1[0] &= START_MASK_OFF;
    }
    if(TBKEY()==0)//BUTTON_RB
    {
        packet2[0] |= RB_MASK_ON;
    } else {
        packet2[0] &= RB_MASK_OFF;
    }
    if(SW1()!=0)
    {
        packet2[0] |= LOGO_MASK_ON;
    } else {
        packet2[0] &= LOGO_MASK_OFF;
    }
}
u8 buttonSemaphore = 0;
u8 packet1=0,packet2=0;
void ButtonTimer1Callback(MultiTimer* timer, void *userData) {
    printf("Button_scan\r\n");
    if (buttonSemaphore == 0) {
        buttonHandler(&packet1,&packet2);
        buttonSemaphore = 1;
    }

    MultiTimerStart(timer, 5, ButtonTimer1Callback, userData);
}
u8 adcSemaphore = 0;
int16_t xtemp = 0, ytemp = 0;
u16 adxsum=0,adysum=0,adcount=0;
void ADCTimer2Callback(MultiTimer* timer, void *userData) {
    printf("ADC Sample\r\n");

    if (adcSemaphore == 0) {

        adysum+=Get_Adc(1);
        adxsum+=Get_Adc(2);

        if(++adcount==10)
        {
            printf("x=%d,y=%d\r\n",adysum/adcount,adxsum/adcount);

            xtemp=(int16_t)map( adxsum/adcount, AD_XMIN, AD_XMAX, INT16_MIN, INT16_MAX );
            ytemp=(int16_t)map( adysum/adcount, AD_YMIN, AD_YMAX, INT16_MAX, INT16_MIN );

            adysum=0;
            adxsum=0;
            adcount=0;
            adcSemaphore = 1;
        }


    }
    MultiTimerStart(timer, 5, ADCTimer2Callback, userData);
}


void JoystickTimer3Callback(MultiTimer* timer, void *userData) {

    printf("Mouse Report\r\n");
    if( USBHD_DevEnumStatus )
    {

        if(adcSemaphore)
        {
            adcSemaphore=0;

            TXData[LEFT_STICK_X_PACKET_LSB] = LOBYTE(xtemp);        // (CONFERIR)
            TXData[LEFT_STICK_X_PACKET_MSB] = HIBYTE(xtemp);
            //Left Stick Y Axis
            TXData[LEFT_STICK_Y_PACKET_LSB] = LOBYTE(ytemp);
            TXData[LEFT_STICK_Y_PACKET_MSB] = HIBYTE(ytemp);
        }
        if(buttonSemaphore)
        {
            buttonSemaphore=0;
            TXData[BUTTON_PACKET_2]=packet2;
            TXData[BUTTON_PACKET_1]=packet1;

        }
        //Clear DPAD
        TXData[BUTTON_PACKET_1] &= DPAD_MASK_OFF;

        USBHD_Endp_DataUp( DEF_UEP1, TXData,sizeof(TXData), DEF_UEP_CPY_LOAD );


    }
    MultiTimerStart(timer, 5, JoystickTimer3Callback, userData);
}
```



### 3.10.3 下载验证

我们把固件程序下载进去，Xbox 360 Controller for Windows出现在电脑上。





## 3.11 实例Eg11_Xinput01

本节目标还是实现Xbox 360 Controller for Windows.

### 3.10.1硬件设计

参考原理图； H2外拓摇杆

### 3.10.2 软件设计

本节在上一节的基础上修改了ADC和按键部分，

首先是ADC代码的配置

```c
// 初始化ADC
// 这里我们仅以规则通道为例
// 我们默认将开启通道0~3
void Adc_Init(void) {
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // 使能ADC1通道时钟和GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);   // 设置ADC分频因子6，将PCLK2分频为12MHz

    // 配置PA1和PA2作为模拟通道输入引脚
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4
            | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;  // 模拟输入引脚
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);  // 复位ADC1

    // 配置ADC1的工作模式
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;       // ADC1和ADC2工作在独立模式
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;              // 模数转换工作在单通道模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;        // 模数转换工作在单次转换模式
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 转换由软件而不是外部触发启动
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;     // ADC数据右对齐
    ADC_InitStructure.ADC_NbrOfChannel = 1;                 // 顺序进行规则转换的ADC通道的数目
    ADC_Init(ADC1, &ADC_InitStructure);  // 根据ADC_InitStruct中指定的参数初始化外设ADC1的寄存器

    ADC_Cmd(ADC1, ENABLE);  // 使能指定的ADC1

    ADC_ResetCalibration(ADC1);  // 使能复位校准

    while (ADC_GetResetCalibrationStatus(ADC1));
    // 等待复位校准结束

    ADC_StartCalibration(ADC1);  // 开启AD校准

    while (ADC_GetCalibrationStatus(ADC1));
    // 等待校准结束

    // ADC_SoftwareStartConvCmd(ADC1, ENABLE);  // 使能指定的ADC1的软件转换启动功能
}

// 获得ADC值
// ch:通道值 0~3
u16 Get_Adc(u8 ch) {
    // 设置指定ADC的规则组通道，一个序列，采样时间
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5); // ADC1,ADC通道,采样时间为239.5周期

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);  // 使能指定的ADC1的软件转换启动功能

    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
    // 等待转换结束

    return ADC_GetConversionValue(ADC1);  // 返回最近一次ADC1规则组的转换结果
}

/*  Re-maps a number from one range to another
 *
 */
int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min,
        int32_t out_max) {
    if (x > in_max) {
        return out_max;
    } else if (x < in_min) {
        return out_min;
    } else {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

}



```

接着是按键代码

```c
#include "Button.h"
/*********************************************************************
 * @fn      buttonGPIOInit
 *
 * @brief   按键IO初始化
 *
 * @param   none
 *
 * @return  none
 */
void buttonGPIOInit(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能GPIOA时钟

    // 配置GPIOA的Pin 0为输入下拉模式（IPD）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置GPIOA的Pin 15为输入上拉模式（IPU）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // 使能GPIOB时钟

    // 配置GPIOB的Pin 3, 4, 5, 6, 7, 8, 9为输入上拉模式（IPU）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6
                                | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}



void buttonHandler(u8* packet1,u8* packet2)
{

    //Button
    if(UPKEY()==0)//Y
    {
        packet2[0] |= Y_MASK_ON;
    }else{
        packet2[0] &= Y_MASK_OFF;
    }
    if(DNKEY()==0)//A
    {
        packet2[0] |= A_MASK_ON;
    }else{
        packet2[0] &= A_MASK_OFF;
    }

    if(LFKEY()==0)
    {
        packet2[0] |= X_MASK_ON;
    } else {
        packet2[0] &= X_MASK_OFF;
    }
    if(RGKEY()==0)
    {
        packet2[0] |= B_MASK_ON;
    } else {
        packet2[0] &= B_MASK_OFF;
    }

    if(BKKEY()==0)//BUTTON_BACK
    {
        packet1[0] |= BACK_MASK_ON;
    } else {
        packet1[0] &= BACK_MASK_OFF;
    }
    if(MDKEY()==0)//BUTTON_LB
    {
        packet2[0] |= LB_MASK_ON;
    } else {
        packet2[0] &= LB_MASK_OFF;
    }
    if(STKEY()==0)//BUTTON_START
    {
        packet1[0] |= START_MASK_ON;
    } else {
        packet1[0] &= START_MASK_OFF;
    }
    if(TBKEY()==0)//BUTTON_RB
    {
        packet2[0] |= RB_MASK_ON;
    } else {
        packet2[0] &= RB_MASK_OFF;
    }
    if(SW1()!=0)
    {
        packet2[0] |= LOGO_MASK_ON;
    } else {
        packet2[0] &= LOGO_MASK_OFF;
    }
    if(SW1()==0)//
    {
        packet1[0] &= L3_MASK_OFF;
    }
    else
    {
        packet1[0] |= L3_MASK_ON;
    }
    if(SW2()==0)//
    {
        packet1[0] |= R3_MASK_ON;
    }
    else
    {
        packet1[0] &= R3_MASK_OFF;
    }
}



```

3. 数据解析

   ```c
   u8 buttonSemaphore = 0;
   u8 packet1=0,packet2=0;
   void ButtonTimer1Callback(MultiTimer* timer, void *userData) {
       printf("Button_scan\r\n");
       if (buttonSemaphore == 0) {
           buttonHandler(&packet1,&packet2);
           buttonSemaphore = 1;
       }
   
       MultiTimerStart(timer, 5, ButtonTimer1Callback, userData);
   }
   u8 adcSemaphore = 0;
   int16_t xtemp = 0, ytemp = 0,Rxtemp=0,Rytemp=0;
   u16 adxsum=0,adysum=0,adRxsum=0,adRysum=0;
   u8 adcount=0;
   void ADCTimer2Callback(MultiTimer* timer, void *userData) {
       printf("ADC Sample\r\n");
   
       if (adcSemaphore == 0) {
   
           adysum+=Get_Adc(1);
           adxsum+=Get_Adc(2);
           adRxsum+=Get_Adc(4);
           adRysum+=Get_Adc(5);
   
           if(++adcount==10)
           {
               printf("x=%d,y=%d\r\n",adysum/adcount,adxsum/adcount);
   
               xtemp=(int16_t)map( adxsum/adcount, AD_XMIN, AD_XMAX, INT16_MIN, INT16_MAX );
               ytemp=(int16_t)map( adysum/adcount, AD_YMIN, AD_YMAX, INT16_MAX, INT16_MIN );
               Rxtemp=(int16_t)map( adRxsum/adcount, AD_YMIN, AD_YMAX, INT16_MAX, INT16_MIN );
               Rytemp=(int16_t)map( adRysum/adcount, AD_YMIN, AD_YMAX, INT16_MAX, INT16_MIN );
               adysum=0;
               adxsum=0;
               adRxsum=0;
               adRysum=0;
               adcount=0;
               adcSemaphore = 1;
           }
   
   
       }
       MultiTimerStart(timer, 5, ADCTimer2Callback, userData);
   }
   
   
   void JoystickTimer3Callback(MultiTimer* timer, void *userData) {
   
       printf("Mouse Report\r\n");
       if( USBHD_DevEnumStatus )
       {
   
           if(adcSemaphore)
           {
               adcSemaphore=0;
   
               TXData[LEFT_STICK_X_PACKET_LSB] = LOBYTE(xtemp);        // (CONFERIR)
               TXData[LEFT_STICK_X_PACKET_MSB] = HIBYTE(xtemp);
   
               TXData[LEFT_STICK_Y_PACKET_LSB] = LOBYTE(ytemp);
               TXData[LEFT_STICK_Y_PACKET_MSB] = HIBYTE(ytemp);
   
               TXData[RIGHT_STICK_X_PACKET_LSB] = LOBYTE(Rxtemp);      // (CONFERIR)
               TXData[RIGHT_STICK_X_PACKET_MSB] = HIBYTE(Rxtemp);
   
               TXData[RIGHT_STICK_Y_PACKET_LSB] = LOBYTE(Rytemp);
               TXData[RIGHT_STICK_Y_PACKET_MSB] = HIBYTE(Rytemp);
   
           }
           if(buttonSemaphore)
           {
               buttonSemaphore=0;
               TXData[BUTTON_PACKET_2]=packet2;
               TXData[BUTTON_PACKET_1]=packet1;
   
           }
           //Clear DPAD
           TXData[BUTTON_PACKET_1] &= DPAD_MASK_OFF;
   
           USBHD_Endp_DataUp( DEF_UEP1, TXData,20, DEF_UEP_CPY_LOAD );
   
   
       }
       MultiTimerStart(timer, 5, JoystickTimer3Callback, userData);
   }
   
   ```

   

### 3.10.3 下载验证

我们把固件程序下载进去，Xbox 360 Controller for Windows出现在电脑上。





## 3.12 实例Eg12_MultiAxisButton

本节目标是实现多轴多按键摇杆

### 3.12.1硬件设计

参考原理图； 

### 3.12.2 软件设计

1. 准备摇杆报表，

```c
/* JoystickRepDesc Report Descriptor */
const uint8_t JoystickRepDesc[ ] =
{
        0x05, 0x01,                                  // Usage Page (Generic Desktop)
        0x09, 0x04,                                  // Usage (Joystick)
        0xA1, 0x01,                                  // Collection (Application)
        0x05, 0x01,                                  // Usage Page (Generic Desktop)
        0x09, 0x01,                                  // Usage (Pointer)
        0xA1, 0x02,                                  // Collection (Logical)
        0x09, 0x30,                                  // Usage (X)
        0x09, 0x31,                                  // Usage (Y)
        0x09, 0x32,                                  // Usage (Z)
        0x09, 0x33,                                  // Usage (Rx)
        0x09, 0x34,                                  // Usage (Ry)
        0x09, 0x35,                                  // Usage (Rz)
        0x09, 0x36,                                  // Usage (Slider)
        0x09, 0x37,                                  // Usage (Dial)
        0x15, 0x00,                                  // Logical Minimum (0)
        0x26, 0xFF, 0x03,                            // Logical Maximum (1023)
        0x75, 0x10,                                  // Report Size (16)
        0x95, 0x08,                                  // Report Count (8)
        0x81, 0x02,                                  // Input (Data,Variable,Absolute)
        0x05, 0x09,                                  // Usage Page (Button)
        0x19, 0x01,                                  // Usage Minimum (Button 1)
        0x29, 0x20,                                  // Usage Minimum (Button 32)
        0x15, 0x00,                                  // Logical Minimum (0)
        0x25, 0x01,                                  // Logical Maximum (1)
        0x75, 0x01,                                  // Report Size (1)
        0x95, 0x20,                                  // Report Count (32)
        0x81, 0x02,                                  // Input (Data,Variable,Absolute)
        0xC0,                                        // End Collection
        0xA1, 0x02,                                  // Collection (Logical)
        0x95, 0x07,                                  // Report Count (7)
        0x75, 0x08,                                  // Report Size (8)
        0x09, 0x01,                                  // Usage (Button 1)
        0x91, 0x02,                                  // Output (Data,Variable,Absolute)
        0xC0,                                        // End Collection
        0xc0                          // END_COLLECTION                                               // End Collection
};
```

2. 最后解析数据。

```c
void JoystickTimer3Callback(MultiTimer* timer, void *userData) {
    printf("Joystick Report\r\n");
    if( USBHD_DevEnumStatus )
    {
        if(adcSemaphore)
        {
            adcSemaphore=0;

            Joystick_Report[0]=ytemp;
            Joystick_Report[1]=ytemp>>8;
            Joystick_Report[2]=xtemp;
            Joystick_Report[3]=xtemp>>8;
            Joystick_Report[4]=ytemp;
            Joystick_Report[5]=ytemp>>8;
            Joystick_Report[6]=xtemp;
            Joystick_Report[7]=xtemp>>8;
            Joystick_Report[8]=ytemp;
            Joystick_Report[9]=ytemp>>8;
            Joystick_Report[10]=xtemp;
            Joystick_Report[11]=xtemp>>8;
            Joystick_Report[12]=ytemp;
            Joystick_Report[13]=ytemp>>8;
            Joystick_Report[14]=xtemp;
            Joystick_Report[15]=xtemp>>8;

        }
        if(buttonSemaphore)
        {
            buttonSemaphore=0;

            Joystick_Report[16]=button;
            Joystick_Report[17]=button;
            Joystick_Report[18]=button;
            Joystick_Report[19]=button;
        }

        if(memcmp(LastJoystick_Report,Joystick_Report,
                        sizeof(Joystick_Report) / sizeof(Joystick_Report[0]))!=0)
        {
            USBHD_Endp_DataUp( DEF_UEP1, Joystick_Report,
                    sizeof(Joystick_Report) / sizeof(Joystick_Report[0]), DEF_UEP_CPY_LOAD );
        }

        memcpy(LastJoystick_Report,Joystick_Report,
                sizeof(Joystick_Report) / sizeof(Joystick_Report[0]));

    }
    MultiTimerStart(timer, 5, JoystickTimer3Callback, userData);
}
```



### 3.12.3 下载验证

我们把固件程序下载进去出现多轴多按键。
