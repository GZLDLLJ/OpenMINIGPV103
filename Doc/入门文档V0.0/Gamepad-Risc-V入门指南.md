# 第一部分、硬件概述

## 1.1 实物概图

![image](https://img2022.cnblogs.com/blog/1966993/202202/1966993-20220217232157176-1998694330.jpg)
 如上图所示，配置了8个6*6轻触按键，一个摇杆（Joystick），搭载一颗WS2812B灯珠，并将UART1串口，编程接口（SWD），外接Joystick接口，microUSB接口引出;
 左边是RKJXV1224005摇杆电位器，右边和下方是8颗6*6的轻触按键，右上方是5050封装的WS2812B灯珠，中间是microusb母座，H3是SWD烧录接口，烧录程序接口，H2是串口，H1是外接摇杆模块的接口；

## 1.2 Gamepad原理图

Gamepad原理图如图1.2所示，如看不清可打开Doc目录下的PDF文档查阅  
![image](https://img2022.cnblogs.com/blog/1966993/202202/1966993-20220217233348812-1046753294.jpg)

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

大家访问以下链接：http://mounriver.com/help

# 第三部分、实战训练

## 3.1 实例Eg1_GamePad

本节我们目标是实现GamePad的功能，枚举成XY轴的平面坐标和8个按键的USB HID类设备。  

### 3.1.1硬件设计

   ![image](https://img2020.cnblogs.com/blog/1966993/202112/1966993-20211214224348752-1607121797.png)
如上图是Joystick原理图，其中VRX1与VRY1是摇杆的电位器输出的电压信号（ADC检测）；SW1则是按键，右侧H1是外接的Joystick口，供接joystick模块使用;  
![image](https://img2020.cnblogs.com/blog/1966993/202112/1966993-20211214224447009-112297073.png)

如上图是KEY原理图，我们只要配置8个GPIO作为输入去检测按键信号;  

### 3.1.2 软件设计

首先是工程树，我们打开工程，可以看到Project Explorer下Gamepad目录如下图
![image](https://img2022.cnblogs.com/blog/1966993/202202/1966993-20220220194934598-1178092890.png)
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

接下来我们来看看main函数，如下

```c
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);

    printf("SystemClk:%d\r\n",SystemCoreClock);
    printf("USBHD Device Test\r\n");

    pEP0_RAM_Addr = EP0_Databuf;
    pEP1_RAM_Addr = EP1_Databuf;
    pEP2_RAM_Addr = EP2_Databuf;

    USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5,ENABLE);
    USB_DeviceInit();
    NVIC_EnableIRQ( USBHD_IRQn );
    ADC_DMA_CONF();
    KEY_INIT();

    while(1)
    {
        printf("X=%d,Y=%d\r\n",ADC_ConvertedValue[0],ADC_ConvertedValue[1]);
        if(Ready)
        {
            Gp_SendReport();
        }
    }
}
```

NVIC_PriorityGroupConfig是配置优先级分组的，Delay_Init初始化延迟函数；USART2_Printf_Init初始化串口打印，
pEP0_RAM_Addr = EP0_Databuf;
pEP1_RAM_Addr = EP1_Databuf;
pEP2_RAM_Addr = EP2_Databuf;
主要配置端点0~2的缓存Ram；
USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5,ENABLE);
配置系统时钟1.5分频，即48M；
USB_DeviceInit()，是对usb设备进行初始化；
ADC_DMA_CONF主要对ADC DMA进行配置；
KEY_INIT是对按键所对应的GPIO进行初始化；
接着是Gp_SendReport，主要是讲处理并上报坐标和按键数据；

```c
void Gp_SendReport(void)
{	
	memset(Joystick_Buf,0,3);
	Ytemp=ADC_ConvertedValue[0];
	Xtemp=ADC_ConvertedValue[1];
	if(Xtemp>Xmax)
        Xtemp=Xmax;
	if(Xtemp<Xmin)
        Xtemp=Xmin;	
	if(Ytemp>=Ymax)
        Ytemp=Ymax;
	if(Ytemp<=Ymin)
        Ytemp=Ymin;
	printf("Xmax=%x,Xcen=%x,Xmin=%x\r\n",Xmax,Xtemp,Xmin);
	printf("Ymax=%x,Ycen=%x,Ymin=%x\r\n",Ymax,Ytemp,Ymin);
	//根据坐标极点确定坐标（两点直线方程）
	X=((Xtemp-Xmin)*255)/(Xmax-Xmin);
	Y=((Ytemp-Ymin)*255)/(Ymax-Ymin);
	Joystick_Buf[0]=X;
	Joystick_Buf[1]=Y;
	Joystick_Buf[2]=Key_Scan();
	Delay_Ms(10);
    while( Endp1Busy )//如果忙（上一包数据没有传上去），则等待。
    { ; }
    Endp1Busy = 1;                                      //设置为忙状态
    memcpy(pEP1_IN_DataBuf, Joystick_Buf, 3);
    DevEP1_IN_Deal(3);
}	
```

最后我们再来看看USBHD_IRQHandler，我们在这个函数值调用了USB_DevTransProcess。


### 3.1.3 下载验证

我们把固件程序下载进去可以，打开“设备与打印机”可以看到USB设备枚举成了一个Gamepad，如下图。  
![image](https://img2022.cnblogs.com/blog/1966993/202203/1966993-20220322204154611-1542025732.png)


右键打开游戏控制器后，点击属性得到下图所示界面  
![image](https://img2022.cnblogs.com/blog/1966993/202203/1966993-20220322204210677-159114354.png)


我们可以摇Joystick和按按键可以发现上图游戏控制器界面也跟着响应。

## 3.2 实例Eg2_Mouse

本节我们目标是实现模拟鼠标的功能，枚举一个具有XY，左右中键以及滚轮上下的功能；  

### 3.2.1硬件设计

同上一章节

### 3.2.2 软件设计

在上一章节的基础上,我们在USB_DevTransProcess中找到报告描述符的获取，并修改为如下内容

```C
case USB_DESCR_TYP_REPORT:
if(((pSetupReqPak->wIndex)&0xff) == 0)     //接口0报表描述符
{
    pDescr = MouseRepDesc;                      //数据准备上传
    len = sizeof(MouseRepDesc);
    Ready = 1;                                  //如果有更多接口，该标准位应该在最后一个接口配置完成后有效
}
else len = 0xff;                                //本程序只有2个接口，这句话正常不可能执行
break;
```

另外鼠标的报告描述符MouseRepDesc如下

```C
const UINT8  MouseRepDesc[]=
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
            0xc0,                          //     END_COLLECTION
            0xc0                           // END_COLLECTION
};
```

然后是报文数据的处理如下：

```C
//处理并上报数据
void Gp_SendReport(void)
{	
	memset(Joystick_Buf,0,4);


	Ytemp=ADC_ConvertedValue[0];
	Xtemp=ADC_ConvertedValue[1];

	if(Xtemp>Xmax)
		Xtemp=Xmax;
	if(Xtemp<Xmin)
		Xtemp=Xmin;	

	if(Ytemp>=Ymax)
		Ytemp=Ymax;
	if(Ytemp<=Ymin)
		Ytemp=Ymin;
	
	printf("Xmax=%x,Xcen=%x,Xmin=%x\r\n",Xmax,Xtemp,Xmin);
	printf("Ymax=%x,Ycen=%x,Ymin=%x\r\n",Ymax,Ytemp,Ymin);
	//根据坐标极点确定坐标（两点直线方程）
	X=((Xtemp-Xmin)*255)/(Xmax-Xmin);
	Y=((Ytemp-Ymin)*255)/(Ymax-Ymin);

    if(X>(X_BASE+20))
    {
        Joystick_Buf[1]=((X-X_BASE)>>DIV)+1;
    }
    if(X<(X_BASE-20))
    {
        Joystick_Buf[1]=(u8)-(((X_BASE-X)>>DIV)+1);
    }
    if(Y>(Y_BASE+20))
    {
        Joystick_Buf[2]=((Y-Y_BASE)>>DIV)+1;;;
    }
    if(Y<(Y_BASE-20))
    {
        Joystick_Buf[2]=(u8)-(((Y_BASE-Y)>>DIV)+1);
    }
    Key_Handle(Joystick_Buf);

    Delay_Ms(5);

    while( Endp1Busy )
    {
        ;                                               //如果忙（上一包数据没有传上去），则等待。
    }
    Endp1Busy = 1;                                      //设置为忙状态
    memcpy(pEP1_IN_DataBuf, Joystick_Buf, 4);
    DevEP1_IN_Deal(4);

}	
```

其中X_BASE为摇杆中点， if(X>(X_BASE+20))就是摇杆左摇动；故而((X-X_BASE)>>DIV)+1计算赋值给我们我们X+坐标；
其他方向同理，
另外Key_Handle的代码如下，主要是为了处理按键与滚轮值

```C
void Key_Handle(uint8_t* kv)
{
    if((LFKEY)==Bit_RESET)
    {
        kv[0]|=0x01;
    }
    if((RGKEY)==Bit_RESET)
    {
        kv[0]|=0x02;
    }
    if(SW1!=Bit_RESET)
    {
        kv[0]|=0x04;
    }
    if((UPKEY)==Bit_RESET)
    {
        if(c_tick++>5)
        {
            kv[3]=1;
            c_tick=0;
        }
    }
    if((DNKEY)==Bit_RESET)
    {
        if(c_tick++>5)
        {
            kv[3]=(u8)-1;
            c_tick=0;
        }
    }
}
```

### 3.2.3 下载验证

我们把固件程序下载进去可以，打开“设备与打印机”可以看到USB设备枚举成了一个“LD Mouse”，如下图。  
![image](https://img2022.cnblogs.com/blog/1966993/202203/1966993-20220325210229360-775600955.png)

我们打开一个网页，摇动摇杆鼠标指针跟着动；右边上下左右键，左右代表鼠标左右键，上下代表滚轮；然后摇杆中键代表鼠标中键；

## 3.3 实例Eg3_KeyBoard

本节我们目标是实现模拟键盘的功能，枚举一个具有Shift键+1~8键的模拟键盘功能；  

### 3.3.1硬件设计

同第一章节

### 3.3.2 软件设计

在上一章节的基础上,我们在USB_DevTransProcess中找到报告描述符的获取，并修改为如下内容

```C
case USB_DESCR_TYP_REPORT:
if(((pSetupReqPak->wIndex)&0xff) == 0)     //接口0报表描述符
{
    pDescr = KeyboardRepDesc;                      //数据准备上传
    len = sizeof(KeyboardRepDesc);
    Ready = 1;             //如果有更多接口，该标准位应该在最后一个接口配置完成后有效
}
else len = 0xff;                                //本程序只有2个接口，这句话正常不可能执行
break;
```

另外Keyboard的报告描述符KeyboardRepDesc如下

```C
const UINT8  MouseRepDesc[]=
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

然后是报文数据的处理如下：

```C
uint8_t Keyboad_Buf[8]={0};
uint8_t lastshift=0,currentshift=0;
uint8_t lastkeycode[8]={0},currentkeycode[8]={0};
static uint8_t KdataFL=0;
//处理并上报数据
void Keyboard_Handle(void)
{	
    memset(Keyboad_Buf,0,8);
    uint8_t i=0;uint8_t idx=2;

    if(SW1==1)
    {
        currentshift|=0x02;
        Keyboad_Buf[0]=currentshift;
    }else{

        currentshift&=(~0x02);
        Keyboad_Buf[0]=currentshift;
    }

    if(UPKEY==0)
    {
        currentkeycode[0]=CODE1;
    }else{
        currentkeycode[0]=0x00;
    }
    if(DNKEY==0)
    {
        currentkeycode[1]=CODE2;
    }else{
        currentkeycode[1]=0x00;
    }
    if(LFKEY==0)
    {
        currentkeycode[2]=CODE3;
    }else{
        currentkeycode[2]=0x00;
    }
    if(RGKEY==0)
    {
        currentkeycode[3]=CODE4;
    }else{
        currentkeycode[3]=0x00;
    }
    if(BKKEY==0)
    {
        currentkeycode[4]=CODE5;
    }else{
        currentkeycode[4]=0x00;
    }
    if(MDKEY==0)
    {
        currentkeycode[5]=CODE6;
    }else{
        currentkeycode[5]=0x00;
    }
    if(STKEY==0)
    {
        currentkeycode[6]=CODE7;
    }else{
        currentkeycode[6]=0x00;
    }
    if(TBKEY==0)
    {
        currentkeycode[7]=CODE8;
    }else{
        currentkeycode[7]=0x00;
    }

    for(i=0;i<8;i++)
    {
        if(currentkeycode[i]!=lastkeycode[i])
        {
            Keyboad_Buf[idx]=currentkeycode[i];
            if(++idx>=8)
            {
                idx=2;
            }
            KdataFL=1;
        }else{
            Keyboad_Buf[idx]=0x00;
        }
    }
    if(currentshift!=lastshift)
    {
        KdataFL=1;
    }

    if(KdataFL!=0)
    {
        KdataFL=0;
        while( Endp1Busy )
        {
            ;                                               //如果忙（上一包数据没有传上去），则等待。
        }
        Endp1Busy = 1;                                      //设置为忙状态
        memcpy(pEP1_IN_DataBuf, Keyboad_Buf, 8);
        DevEP1_IN_Deal(8);

    }
    Delay_Ms(5);
    memcpy(lastkeycode,currentkeycode,8);
    lastshift=currentshift;
}
```

最后是main函数，只改了while中的Keyboard_Handle();

```C
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Return         : None
*******************************************************************************/
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);

    printf("SystemClk:%d\r\n",SystemCoreClock);
    printf("USBHD Device Test\r\n");

    pEP0_RAM_Addr = EP0_Databuf;
    pEP1_RAM_Addr = EP1_Databuf;
    pEP2_RAM_Addr = EP2_Databuf;

    USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5,ENABLE);
    USB_DeviceInit();
    NVIC_EnableIRQ( USBHD_IRQn );
    ADC_DMA_CONF();
    KEY_INIT();

    while(1)
    {
        printf("X=%d,Y=%d\r\n",ADC_ConvertedValue[0],ADC_ConvertedValue[1]);
        if(Ready)
        {
            Keyboard_Handle();
        }
    }
}
```

### 3.3.3 下载验证

我们把固件程序下载进去可以，打开“设备与打印机”可以看到USB设备枚举成了一个“LD Keyboard”，如下图。  
![image](https://img2022.cnblogs.com/blog/1966993/202203/1966993-20220329215629544-759735995.png)

我们打开一个键盘测试网页，地址如下：
<https://keyboard.bmcx.com/>
按摇杆按键SW1即为shift键按下，其他键分别对应主键盘的1~8；shift+1-8键也可以组合；



## 3.4 实例Eg4_WS2812B

本节我们目标是实现点亮WS2812B LED，开机点亮红绿蓝灯后关灯，再次点亮是七彩渐变效果；  

### 3.4.1硬件设计

参考原理图LED部分；

### 3.4.2 软件设计

首先是WS2812B的初始化WS281xInit，Delay_Ms(50)是为了延迟等待灯源稳定；WS_WriteAll_RGB是设置所有LED为同一种颜色；

TIM3_PWMOut_Init是配置PA6为TIM3 channel1为PWM1 OUT模式，0分频60个周期；TIM3_DMA_Init是DMA的配置；  

 “TIM_DMACmd(TIM3, TIM_DMA_Update, ENABLE);TIM_Cmd(TIM3, ENABLE);TIM_CtrlPWMOutputs(TIM3, ENABLE);”这三个函数是使能DMA和PWM输出；

```C
void WS281xInit(void)
{
    Delay_Ms(50);
    WS_WriteAll_RGB(0x7f,0,0);//set red
    TIM3_PWMOut_Init(60, 0, 0);
    TIM3_DMA_Init(DMA1_Channel3, (u32)TIM3_CH1CVR_ADDRESS, (u32)&send_Buf, NUM);

    TIM_DMACmd(TIM3, TIM_DMA_Update, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    Delay_Ms(500);
    WS_WriteAll_RGB(0,0x7f,0);//set green
    Delay_Ms(500);
    WS_WriteAll_RGB(0,0,0x7f);//set blue
    Delay_Ms(500);
    WS_WriteAll_RGB(0,0,0x00);//set black
    Delay_Ms(500);
}
```

调用上面的函数可以实现WS2812B的初始化；并点亮红绿蓝灯后关灯；接着我们要实现七彩渐变的效果；

```C
void Colorful_Gradient_Pro(void)
{
    switch(BreathType)
    {
        case 0://red
            {
                R_duty=85;
                G_duty=1;
                B_duty=1;
                Swich_TIME=0;
                BreathType=1;
            }break;
        case 1://red->orange
            {
                R_duty-=1;
                G_duty+=2;
                if(++Swich_TIME>=PWM_TICK_BASE)
                {
                    Swich_TIME=0;
                    BreathType=2;
                }
            }break;
        case 2://orange
            {
                R_duty=75;
                G_duty=23;
                B_duty=0;
                Swich_TIME=0;
                BreathType=3;
            }break;
        case 3://orange->yello
            {
                G_duty+=5;
                if(++Swich_TIME>=PWM_TICK_BASE)
                {
                    Swich_TIME=0;
                    BreathType=4;
                }
            }break;
        case 4://yello
            {
                R_duty=74;
                G_duty=75;
                B_duty=1;
                Swich_TIME=0;
                BreathType=5;
            }break;
        case 5://yello->green
            {
                R_duty-=7;
                B_duty+=1;
                if(++Swich_TIME>=PWM_TICK_BASE)
                {
                    Swich_TIME=0;
                    BreathType=6;
                }
            }break;
        case 6://green
            {
                R_duty=0;
                G_duty=75;
                B_duty=9;
                Swich_TIME=0;
                BreathType=7;
            }break;
        case 7://green->cyan
            {
                B_duty+=6;
                if(++Swich_TIME>=PWM_TICK_BASE)
                {
                    Swich_TIME=0;
                    BreathType=8;
                }
            }break;
        case 8://cyan
            {
                R_duty=0;
                G_duty=75;
                B_duty=72;
                Swich_TIME=0;
                BreathType=9;
            }break;
        case 9://cyan->blue
            {
                G_duty-=7;
                B_duty+=1;
                if(++Swich_TIME>=PWM_TICK_BASE)
                {
                    Swich_TIME=0;
                    BreathType=10;
                }
            }break;
        case 10://blue
            {
                R_duty=4;
                G_duty=1;
                B_duty=82;
                Swich_TIME=0;
                BreathType=11;
            }break;
        case 11://blue->purple
            {
                R_duty+=7;
                B_duty-=1;
                if(++Swich_TIME>=PWM_TICK_BASE)
                {
                    Swich_TIME=0;
                    BreathType=12;
                }
            }break;
        case 12://purple
            {
                R_duty=75;
                G_duty=1;
                B_duty=67;
                if(++Swich_TIME>=PWM_TICK_BASE)
                {
                    Swich_TIME=0;
                    BreathType=13;
                }
            }break;
        case 13://purple->red
            {
                R_duty+=1;
                B_duty-=6;
                if(++Swich_TIME>=PWM_TICK_BASE)
                {
                    Swich_TIME=0;
                    BreathType=0;
                }
            }break;

    }
    WS_WriteAll_RGB(R_duty,G_duty,B_duty);
    Delay_Ms(100);
}
```

调用以上Colorful_Gradient_Pro即可实现WS2812的七彩渐变效果；

最后是main函数，在配置初始化中调用WS281xInit；Colorful_Gradient_Pro在while中循环调用；

```C
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n",SystemCoreClock);
    printf("WS2812B LED Test\r\n");
    WS281xInit();
    while(1)
    {
        Colorful_Gradient_Pro();
    }
}
```

### 3.4.3 下载验证

我们把固件程序下载进去可以，看板载的WS2812B灯珠， 开机点亮红绿蓝灯后关灯，再次点亮是七彩渐变效果 完美显示；

### 3.4.4 参考资料

我们写代码过程中参考了以下博主的资料，在此感谢和致敬。

STM32使用LL庫PWM的DMA模式驅動ws2812：<https://chowdera.com/2022/03/202203010649484502.html>



## 3.5 实例Eg5_CompositeDevice

本节我们目标是实现复合设备；一个复合了Gamepad、Mouse、Keyboard的设备；  

### 3.5.1硬件设计

参考原理图部分；

### 3.5.2 软件设计

首先我们先看main函数，main函数里面主要进行了USB初始化，TIM2的初始化，KEY的初始化，ADC的初始化，WS2812B的初始化 ；

```C
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("Compile Time: %s  %s\n", __DATE__, __TIME__);

    pEP0_RAM_Addr = EP0_Databuf;
    pEP1_RAM_Addr = EP1_Databuf;
    pEP2_RAM_Addr = EP2_Databuf;
    pEP3_RAM_Addr = EP3_Databuf;
    USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5,ENABLE);
    USB_DeviceInit();
    NVIC_EnableIRQ( USBHD_IRQn );
    TIM2_Int_Init(47,999);
    KEY_INIT();
    ADC_DMA_CONF();
    printf("WS2812B LED Test\r\n");
    WS281xInit();
    while(1)
    {
        TaskProcess();
    }
}
```

其中，上面的TIM2_Int_Init配置为1ms的定时器更新，主要是为了下面的定时轮询任务系统作为基准tick；

```C
#include "PollSystem.h"
#include "ws281x.h"
#include "Keyboard.h"
#include "myFSM.h"
#include "gamepad.h"
#include "Mouse.h"
static TASK_COMPONETS TaskComps[] =
{
	{0,100,100,WS281x_Handle},	//1
	{0,10,10,FSM_Handle},               //2
	{0,5,5,Gamepad_Handle},            //3
	{0,5,5,Keyboard_Handle},            //4
	{0,5,5,Mouse_Handle}            //5
};
//在main-While循环中调用
void TaskProcess(void)
{
	uint8_t i;
	for (i=0; i<TASKS_MAX; i++) // 逐个任务时间处理
	{
		if (TaskComps[i].Run) // 时间不为0
		{
			TaskComps[i].TaskHook(); // 运行任务
			TaskComps[i].Run = 0; // 标志清0
		}
	}
}
//在定时器中断中调用
void TaskRemarks(void)
{
	uint8_t i;
	for(i=0;i<TASKS_MAX;i++)
	{
		if(TaskComps[i].Timer)
		{
			TaskComps[i].Timer--;
			if(TaskComps[i].Timer==0)
			{
				TaskComps[i].Timer=TaskComps[i].ItvTime;
				TaskComps[i].Run=1;
			}
		}
	}
}
```

按照注释调用TaskProcess和TaskRemarks，TaskProcess是为了判断任务时间是否到了，并运行任务，清空运行标志；

TaskRemarks是对各个任务的时间片进行处理；static TASK_COMPONETS TaskComps[] 这个数组管理的是各个任务，

数据结构如下：

```C
typedef struct _TASK_COMPONETS
{
	uint8_t Run;//程序运行标记: 0-不允许，1运行
	uint16_t Timer;//计时器
	uint16_t ItvTime;//任务运行间隔时间
	void (*TaskHook)(void);//任务定义
	
}TASK_COMPONETS;
```

如上面结构体所示，Run是任务运行标记；Timer是任务的的计时器，ItvTime是任务运行间隔时间，Timer为零是即重载ItvTime时间间隔；TaskHook是任务的函数指针；再回过头看看以下代码：

```c
static TASK_COMPONETS TaskComps[] =
{
	{0,100,100,WS281x_Handle},	//1
	{0,10,10,FSM_Handle},               //2
	{0,5,5,Gamepad_Handle},            //3
	{0,5,5,Keyboard_Handle},            //4
	{0,5,5,Mouse_Handle}            //5
};
```

WS281x_Handle根据设备类型进行LED颜色设置，Gamepad设备亮红灯，Mouse设备亮绿灯，KeyBoard设备亮蓝灯，100ms刷一次数据，代码如下：

```C
void WS281x_Handle(void)
{
    if(myfsm.hiddev==Gamepad)
    {
        WS_WriteAll_RGB(0x7f,0,0);
    }else if(myfsm.hiddev==Mouse){
        WS_WriteAll_RGB(0,0x7f,0);
    }else if(myfsm.hiddev==KeyBoard){
        WS_WriteAll_RGB(0,0,0x7f);
    }

}
```

FSM_Handle是状态机处理函数，主要功能是切换设备；按住SW1摇杆电位器按键进行计数，500个tick也就是5S后进行切换设备；

```C
void FSM_Handle(void)
{
    if(SW1!=Bit_RESET)
    {
        if(myfsm.swtick++>500)//5S=5000ms=500tick
        {
            myfsm.swtick=0;
            if(((uint8_t)++myfsm.hiddev)>2)
            {
                myfsm.hiddev=Gamepad;
            }
        }
    }else{
        myfsm.swtick=0;
    }
}
```

Gamepad_Handle，Mouse_Handle，Keyboard_Handle是对应设备的数据处理和上报；大家参考之前的章节；

### 3.5.3 下载验证

我们把固件程序下载进去，
默认是Gamepad，实验现象同3.1.3，亮红灯；

按住SW1 5S后是Mouse，实验现象同3.2.3，亮绿灯；

按住SW1 5S后是Keyboard，实验现象同3.3.3，亮蓝灯；

### 3.5.4 参考资料

我们写代码过程中参考了以下博主的资料，在此感谢和致敬。

https://blog.csdn.net/weixin_44576486/article/details/108908613

