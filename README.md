# LZU_ASCENTO
毕业设计：模仿ascento，一种四连杆轮腿机器人(论文地址：https://ieeexplore.ieee.org/document/8793792)<br>
代码参考达妙开源桌面轮足https://gitee.com/kit-miao/balance_robot
## 所需软件
clion+stm32cubemx(b站keysking教程) soildworks(教程自学，整车结构不开源，因为画的有点烂，最关键的就是腿结构) matlab vscode 嘉立创eda
## bom表及成本
1.stackforce关节电机（单编，额定5N.M）*2->900元（更推荐买达妙电机）<br>
2.dmh6215轮毂电机*2->650元<br>
3.radiomaster遥控器+elrs接收机->400元<br>
4.达妙dmmc02开发板（stm32h723）->200元<br>
5.博氏航模锂电池（还要额外购买平衡充，120W电源充电器）->450元<br>
6.m3和m5螺丝，螺母，法兰轴承，垫片，束线管若干->100元<br>
7.打印件，cnc加工，轮胎胎皮（我的是江昕5*2A轮胎）->400元<br>
（讲一下如何选型电机：机器人关节一般都是关节减速电机，你会看到不同电机扭矩参数，复刻中小型轮腿机器人推荐额定扭矩3N-10N，robomaster那种都要20N+,30N+,轮电机的话，小轮腿比如达妙桌面轮足选用dmh3510,额定扭矩0.18N.M,我的毕设，轮毂电机额定是1N.M，你还可以选本末电机）
## 代码文件简介
1.core/inc:若干头文件<br>
2.core/src/task:freertos两个任务，分别是姿态解算，底盘任务<br>
3.core/src/bsp:bmi088陀螺仪，can过滤器配置及回传数据，遥控器elrs协议，遥控器通道功能分配，电机控制MIT协议<br>
4.core/src/filtering....:滤波算法，卡尔曼,mahony,扩展卡尔曼<br>
（着重强调代码调试一定要弄清电机正方向，一定要弄清电机正方向，一定要弄清电机正方向，一般逆时针为正！！！）
## 手搓记录：
2025.9-12：分电板，solidworks建模<br>
2026.1.6：学习freertos(b站keysking)，学习can总线，成功驱动关节电机与轮毂电机<br>
2026.1.7-2.1:阶段性摆烂，看两篇论文(ascento,轮腿式平衡机器人控制），学习了一下动力学模型，与LQR理论<br>
2026.2.3：解决轮毂电机震荡，问题出在没有深入理解mit控制协议，其中kp,kd相关<br>
2026.2.4：一开始想着先调腿，看b站视频说要先调轮，再调腿，更换工作方向<br>
2026.2.5：电机威力太大，把打印件弄碎了，问题出在没有做应力分析，简单修改了一下模型重新打印<br>
2026.2.6：发现can1数据收发正常，can2发送正常，但是接收不到数据，问题出在cubemx配置时RxFifo1ElmtsNbr没有给值<br>
2026.2.7：LQR控制部署到stm32，没反应，问题出来轮毂电机正负方向没弄清楚<br>
2026.2.10：调试几天车没有起色，一直震荡，发现代码中陀螺仪角速度方向弄反了,纠正后LQR起效了，但很难稳住<br>
2026.2.11：腿伸长时，由于机身较重，导致轮外八，影响运动，现在修改模型，改为轮子内扣<br>
2026.2.12：内扣依然同样问题，开学整cnc加工腿部<br>
2026.2.13-3.1：过年摆烂，卡在遥控器接收crsf协议，串口空闲中断+DMA一直收不到数据<br>
2026.3.1-3.10：更换扭矩更大的轮毂电机，现在更稳了，但是轮腿外八现象依然存在。而且遥控器数据一直接收不到，期间还下单cnc加工腿部<br>
2026.3.11:突然发现一篇文章https://blog.csdn.net/qq_41544116/article/details/100155203
然后我打印printf("elrs_data_temp addr: 0x%08X\r\n", (uint32_t)elrs_data_temp);发现结果elrs_data_temp addr: 0x20008130，于是找到问题。将缓冲区显式放入AXI SRAM（地址0x24000000）。修改链接脚本STM32H723XX_FLASH.ld，在SECTIONS中添加一个新段：<br>
.dma_buffer (NOLOAD) : {<br>
    *(.dma_buffer)<br>
} > RAM<br>
其中RAM已在MEMORY中定义为ORIGIN = 0x24000000的AXI SRAM区域。在代码中定义数组时指定该段：uint8_t elrs_data_temp[64] __attribute__((section(".dma_buffer")));<br>
然后可以正常接收数据了<br>
2026.3.12:分配遥控器通道功能，现在可以实现遥控变腿长，前进后退，转向pd控制还在调
