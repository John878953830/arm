# INNFOS SCA Controller Ref Design
*  Copyright 2019 - 2022 INNFOS (Beijing) Technology Co., Ltd.
*  www.innfos.com
*  All rights reserved.
*  推荐将编辑器的缩进参数和TAB设置为4 来阅读本文件
*  发布日期:2019年9月10日
*  版本：V1.5.2
---
## 一、程序说明
1、该程序实现了通过CAN总线对INNFOS 执行器的控制功能，包括了所有的操作API；</br>
2、该程序所运行的MCU为：STM32F429IGT6，可以方便的移植到其他的STM32F429芯片中，和其他系列的MCU中；</br>
3、该程序开发平台为 `Keil MDK V5.21.1.0`。</br>

## 二、主要文件架构（SCA文件夹内）
1、`SCA_Protocol.c/h`：INNFOS CAN 通信协议层，该协议层完成了数据帧封装，解包等步骤，使用CAN端口进行数据收发；</br>
2、`SCA_API.c/h`：通信协议层的封装，包含了所有参数的读写API；</br>
3、`SCA_APP.c/h`：演示程序；</br>
4、`bsp_can.c/h`：STM32 CAN底层驱动程序，波特率固定1Mbps，位于BSP目录下；</br>

## 测试准备工作
1、将程序下载至 STM32F429 单片机，若不同平台则需对程序进行移植;</br>
2、使用USB转串口连接至单片机的串口1（默认 PA9 PA10），波特率`115200`；</br>
3、打开虚拟串口终端，发送16进制数字 6 将收到串口打印出来的提示信息，所有数据以16进制发送，ASC码形式显示；</br>
4、根据提示信息进行测试。</br>
</br>

#  更新日志
*  `V1.5.2` 修复对旧版编译器的兼容性。

*  `V1.5.1` 增加总线轮询功能，用于查找并显示总线上存在的执行器ID。

*  `V1.5.0` 更改数据收发机制，大部分API可选用阻塞或非阻塞的执行方式。协议层加入统一数据接收接口canDispatch(CanRxMsg* RxMsg)，在有新数据接收时
进行调用。本版本取消了查询式接收，改为中断接收，也可以在RTOS中开启单独任务接收数据。控制器初始化流程改为手动执行，取消自动轮询初始化。加入了CAN端口配置，可同时利用多个CAN端口对执行器进行控制，本例程开启了MCU的CAN1控制器和CAN2控制器,分别控制两台执行器。

* `V1.1.0` 增加所有参数的读写API，函数名称与PC SDK对齐，更新协议层代码。

*  `V1.0.0`	初版程序，采用阻塞式执行方式，即发送数据后等待返回。