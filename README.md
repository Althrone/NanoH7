# NanoH7
针对stm32h750进行飞控开发。Editor: VSCode, Compiler: arm-none-eabi-gcc, Programmer: OpenOCD, Debugger: OpenOCD. STM32 open source development method.

工具链：Windows-VSCode-arm-none-eabi-gcc-make-OpenOCD

这个工程最主要的目的是对stm32芯片进行轻量化开发，现在多数初学者开发单片机都是不同芯片装不同IDE，体积臃肿，而且工作后如果从事这一行有可能被查水表。使用该方法可以使得程序全部在VSCode上编辑，只需要安装不同的编译器、烧录器和调试器即可。

VSCode不说了，最强IDE(雾。

arm-none-eabi-gcc其实就是一个开源的编译器，用这个编译器c程序在arm芯片上进行裸机开发。[GNU Arm Embedded Toolchain Downloads](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)`←鼠标中键点击以打开新窗口`

OpenOCD是一个开源的片上调试器，支持非常多芯片，同时也有烧录功能，这个工程的程序烧录也是靠它做的。

## 文件树
├─.vscode（vscode的一些设置）  
├─build（编译后的.o文件和二进制文件都放这里）  
├─rt-thread（源码所在）  
│  ├─bsp\stm32（板级支持包）  
│  │  ├─libraries（rtt驱动库和hal驱动库）  
│  │  └─rcf-nano-h7（我的板子的代码）  
│  ├─components（rtt组件）  
│  │  ├─dfs（文件系统）  
│  │  ├─drivers（外设驱动）  
│  │  └─finsh（终端显示组件）  
│  ├─include（rtt相关）  
│  ├─libcpu\arm\cortex-m7（rtt内核级）  
│  └─src（rtt核心源码）  
└─其他文件

## 更新进度
|时间|更新内容|
|-|-|
|2022/8/13|简单编译工程，各模块都加进来了|
|2024/4/14|1 添加了板载传感器的线程，但是并不能2.5ms执行一次，088不知道发什么神经，回去看看int2引脚输出啥情况<br />2 遥控器能读到数据，但是粘包，也是很傻逼<br />3 gps只能收到第一个字节，服了<br />4 虚拟串口会导致死机，也不知道啥情况<br />太难了|
|2024/4/15|解决了ins线程的定时问题，现在可以利用tim_cbk顺便获取部分acc值，imu获取值总耗时170us左右|

## 088特性
如果在读取ACC_INT_STAT_0寄存器的时候，里面的值才发生更新，读出结果还是未更新的值，但是寄存器会被直接清零

高实时性任务参见代码实现方式
imu 2.5ms更新一次 mag 5ms更新一次

## 5983特性
无法读取cr0-cr3的值
除非芯片断电，进入连续模式后芯片软复位也无法停止连续模式

## ins板子
由于三四线混合，spl06最高只能支持到6m(带一小段飞线，不接逻辑分析仪)