# STM32F103C8T6 学习

看的教程是野火：

- 视频：[【野火】STM32 HAL库开发实战指南 教学视频](https://www.bilibili.com/video/BV18X4y1M763/)
- 书籍：[[野火]STM32 HAL库开发实战指南——基于F103系列开发板](https://doc.embedfire.com/mcu/stm32/f103/hal_general/zh/latest/index.html)

书跟视频内容是一样的，前期感觉可以看看视频进入下状态，后面嫌视频速度慢就可以直接看书了。

野火的 HAL 库这套教程看时间是 2021 年出的，在这之前野火的教程讲的都是标准外设库（standard peripheral library），而且不同的开发版都分别有不同的视频与书籍进行讲解，大部分内容是一致的，但是有很多小区别。

现在都建议看 HAL 库的视频了，因为相比旧的外设库，HAL 库有如下优势：

- 可移植性更高，STM32 全系列的代码都可以通用，不需要做修改。
  - 也因此野火这一套新教程就 Cover 了自己所有的开发版，不再需要像以前一样每个板子都录一遍视频了。
- 因为底层已经做了一层完全的抽象，上层代码都通用了，HAL 上层为我们提供了更丰富、更复杂的库函数，供我们使用，这能大大提升开发效率。
- STM32 还推出了基于 HAL 的 STM32CubeMX，这是一个代码生成器，可以随便点两下就能生成一套按需定制的样板代码，相当方便。

仔细看的话也能发现野火自己的视频里，STM32 系列就属 HAL 库这个视频播放量最高，所以入门就看它没错的。

另一方面标准外设库目前已经彻底停止更新了，所以现在基本都得用 HAL 库或者第三方的库。

>实际我是看了大半野火 18 年出的 F103 教程，然后才搞明白原来 HAL 这个是最新的，也是入门视频...不过问题不大，内容实际都差不多。

## 开发板

用的开发版是某宝上 13 元一块的 STM32F103C8T6 最小系统板。

板子原理图如下：

![](_img/STM32F103C8T6-board-prints.webp)

PCB 结构尺寸图如下：

![](_img/stm32f103c8t6-pcb-prints.webp)

STM32 官方的两本手册，日常写程序都需要参照的：

- 数据手册 STM32F103xB datasheet: <http://www.st.com/stonline/products/literature/ds/13587.pdf>
- 参考手册 RM0008 Reference manual: <http://www.st.com/stonline/products/literature/rm/13902.pdf>

## Platformio 开发环境

因为用的是 Linux，入门阶段仍然选用 PlatformIO 作为 IDE 玩耍。

PlatformIO 上支持 stm32 的 framework 有好几个：

1. Arduino: 貌似是 platformio 上最流行的一个 platform，esp32 也可以用，但是还不太了解。
2. [CMSIS](https://github.com/ARM-software/CMSIS_5)(The ARM Cortex Microcontroller Software Interface Standard): ARM 官方推出的一个 Cortex-M 系列处理器的硬件抽象层，各 SOC 之间都通用，不过比较底层。
5. STM32 Standard Peripheral Libraries: 这就是前面提到的 ST 标准外设程序库，野火旧的教程都是基于这个库来讲解的。
   1. 这个库实际也是基于 CMSIS 实现的，而且 ST 官方早就不更新了，现在官方主推 HAL 库，所以现在不建议使用此库了。
6. [STM32Cube](https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer): 这也是 ST 官方开源的一套软件库，包含 HAL 硬件抽象层。别人如果说 STM32 HAL，那就是指这个。
7. [libopencm3](https://github.com/libopencm3/libopencm3): Cortex-M0(+)/M3/M4 系列控制器的开源固件库
8. 其他 RTOS 相关库，先忽略。

貌似目前入门都是直接推荐 STM32Cube HAL 了，所以我选了这个。

## 烧录

我这个最小系统板的 micro usb 接口是没有 ttl 转 usb 功能的，另一侧的 SWIO/SWCLK 这里虽然看着也是 4 个引脚，但是它是给仿真器用的 SWD 接口。

JTAG/SWD 接口都是一种可用于调试、下载程序的接口，需要配合对应的仿真器使用。

查看此项目的 `platformio.ini` 配置，其中有 `upload_protocol`，支持多种协议，简要介绍其中常用的几种：

- serial: 即 TTL 串口，最原始的固件上传协议，上传速度比较慢，而且不支持调试。
- cmsis-dap: 是 ARM 官方的调试协议规范
  - 此标准最流行的用法是开源项目 [ARMmbed/DAPLink](https://github.com/ARMmbed/DAPLink)，它将一块 STM32 板子当成调试器，通过 CMSIS-DAP 协议来调试另一块板子。 
- st-link: ST 意法半导体的商业调试器，主要用在 STM32 相关板子上，因为 STM32 的流行而开始流行。
- j-link: 老牌闭源商业调试器，据说性能很好，不过正版的贼贵，没啥必要整。

其中串口使用 TX/RX 两根数据线进行异步通信，而其他调试器一般都使用 JTAG 或 SWD 接口与被调试开发版连接，再通过 USB 线与主机通信。

作为开源爱好者，我选择开源的 DAPLink 调试器。这里介绍下 DAP 跟 serail 两种烧录方式，而 DAP 调试会在后面适合的时候讲到。

### 1. Serail 串口烧录

我之前已经玩过一波 ESP8266 跟 51 单片机了，对 TTL 串口烧录比较熟悉。它缺点就是速度比较慢，而且不能用于调试。

根据各种文章介绍，STM32 基本全系列都是用 PA9 PA10 这两个引脚进行串口通信：

- 串口线的 RX 接 STM32 的 PA9，对应我开发板的 A9 引脚
- 串口线的 TX 接 STM32 的 PA10，对应我开发板的 A10 引脚
- 当然接地线也得连一下，随便找个 GND 引脚接一下就行。
- 供电线可连可不连，看你是怎么供电的了。如果是用了别的电源，要注意将两个电源的 GND 接一起，统一 GND 的电位。

然后下载前，还需要对 BOOT 做调整。这两个

| BOOT0 | BOOT1 | MODE  |
| ----- | ----- | ----- |
| 0     | X(0/1 都可)     | 用户 FLASH |
| 1     | 1     | 内部 SRAM  |
| 1     | 0     | 系统存储器（包含出厂预置的 bootloader）   |

其中系统存储器包含了出厂预置的 bootloader，我们需要通过它实现串口自动下载程序，因此需要对 BOOT 做如下调整：

- BOOT0 接 3.3V，即 1
- BOOT1 接 GND，即 0

最后，platformio 对 stm32 平台，默认使用 stlink 进行上传，但是我们现在要使用串口，所以要在 `platform.ini` 中添加 `upload_protocol` 相关参数，修改后内容大致如下：

```ini
[env:learn-STM32F103C8]
platform = ststm32
board = genericSTM32F103C8
framework = stm32cube
# AVAILABLE upload_protocol: blackmagic, cmsis-dap, dfu, jlink, serial, stlink
upload_protocol = serial
```

这样完成后，就可以将 USB 转 TTL 线插入电脑，用 platformio 进行程序烧录了（如果板子一直有上电，还需要按下 RESET 键）。

>串口设备在 Linux 系统中通常被命名为 `/dev/ttyUSB0`.

烧录完成后，需要将 BOOT0 与 BOOT1 都接到 0，再按 RESET 键才能正常启动。

### 2. DAP 调试器烧录

>也有叫它 DAP 仿真器的，但是没理解为什么这么翻译...

前面介绍了调试器一般使用  JTAG 或 SWD 接口与开发板通信，在使用 STM32CubeMX 生成项目时也能在调试参数中找到多个选项，可以通过调整选中项，观察 STM32CubeMX 右侧芯片引脚的变化。

以 STM32F103C8T6 及其引脚全兼容的国产替代品为例，介绍下其中四种调试模式的差别：

- Serial Wire: 即通过 SWD(Serial Wire Debug) 接口进行调试，仅需要 PA13(SWD_DIO)、PA14(SWD_CLK) 两个引脚 + 任意一个其他引脚提供 RESET 信号。
- Trace Asynchronous SW: 在 Serial Wire(SWD) 的基础上添加了一根 SWO 引脚，实现了串行数据输出，所以加上 RESET 需要 4 根线。
- JTAG 4 pins: 使用 TMS、TCK、TDI、TDO 四根调试信号引脚 + 任意一根 RESET 引脚。
- JTAG 5 pins: 在 JATG 4 pins 的基础上添加了一根 TRST 引脚。（功能说明我没看懂...）

而我这里已经有了一块自带 DAPLink 固件的合宙 AIR32F103CBT6，被我用来当 DAP 调试器用，其官方文档 [合宙 AIR32CBT6 使用说明](https://wiki.luatos.com/chips/air32f103/board.html) 介绍了 SWD 模式下的接线方式：

| AIR32F103CBT6 调试器引脚   | 被调试设备引脚 |
| ----    | ----------------  |
| PB13    | SWD_CLK，对应 STM32F103C8T6 的 PA14      |
| PB14    | SWD_DIO，对应 STM32F103C8T6 的 PA13      |
| PB0     | RST 复位，对应任一未使用到的引脚      |
| PA2（模拟的串口 TX）     | 串口 RX，通常对应 PA10，具体得看程序的设置       |
| PA3（模拟的串口 RX）     | 串口 TX，通常对应 PA9，具体得看程序的设置       |
| VCC/GDN  | VCC/GDN     |

>不接 TX/RX 两根串口线也可以上传调试，但是就没法通过串口进行交互（查看被调试设备的输出、发送消息给被调试设备）。

>离谱的是，我用 STM32CubeMX 生成代码时，一定得用 JTAG 模式，才能正常上传、调试...换成 Serial Wire 的话，断电重启后就没法上传了，得改 BOOT 跳帽通过 bootloader 重刷...暂时还没理解原因...

照着这个列表接好线即可，一共是 7 根线。

然后是改 platformio 配置，跟 TTL 一样，首先还是改 `platform.ini` 添加 `upload_protocol`，但是 DAP 还支持调试功能，因此还可以额外添加 `debug_tool` 指定调试器协议，改完效果大致如下：

```ini
[env:learn-STM32F103C8]
platform = ststm32
board = genericSTM32F103C8
framework = stm32cube
# AVAILABLE upload_protocol: blackmagic, cmsis-dap, dfu, jlink, serial, stlink
upload_protocol = cmsis-dap
# 调试协议也设为 DAP
debug_tool = cmsis-dap
# 波特率要设置得与被调试设备一致，否则输出内容会乱码
monitor_speed = 115200
```

这样完成后，就可以通过 USB 连接 DAP 调试器与电脑，用 platformio 进行程序烧录了。

烧录完成后，需要将 BOOT0 与 BOOT1 都接到 0，再按 RESET 键才能正常启动。

>这里可以看到，使用 DAP 调试器还有个好处，就是不需要调整 BOOT0/BOOT1 跳帽，通过系统存储器中的 bootloader 来上传数据，省心不少。

## 芯片的输入输出结构

省略若干内容...

GPIO 的两种输出结构（野火教程使用了三极管来简单说明）：

- 推挽(push-pull)输出: 输出能力较强
- 开漏输出: I2C 使用了此输出结构

详细原理之后仔细学电子电路再了解，先 pass.


## 一、LED 闪烁

可在 `1_led_blink` 中测试此这一节的代码逻辑。

### 1. 第一个程序 - 通过 GPIO 点亮开发板上自带的 LED 测试灯

测试灯旁边的标记是 PC13，结合原理图可知此 LED 的一端接的 3V3 电源，另一端接的是 STM32 的 PC13 引脚，对应 GPIO13。

这里第一个程序通过控制 GPIO13 点亮小灯。为了理解原理，我们直接通过操作最底层的寄存器地址来点灯。

>注：这里使用了 `UL` 后缀表示 unsigned long，根据  [ARM 官方文档 - Basic data types in ARM C and C++](https://developer.arm.com/documentation/dui0375/g/C-and-C---Implementation-Details/Basic-data-types-in-ARM-C-and-C--)，unsigned long 与 unsigned int 都是 4bytes，完全可互换

```c
#include <stm32f103xb.h>

// 根据参考手册 3.2 节，Port C 对应的寄存器地址为: 0x40011000UL
// 再根据参考手册 9.2.4 节，配置 GPIO Output 的寄存器地址偏移量为 0x0C，这样就得到 GPIO 输出寄存器的绝对地址为 0x4001100CUL
#define GPIOC_OUTPUT_PTR (unsigned int *)0x4001100CUL

// 再根据参考手册 9.2.2 一节，配置 GPIO 高位的寄存器地址偏移量为 0x04，这样就得到 GPIO 配置寄存器的绝对地址为 0x40011004UL
#define GPIOC_CONFIG_PTR (unsigned int *)0x40011004UL

// 根据参考手册 3.2 节，RCC 时钟控制寄存器的地址为: 0x40021000UL，且 GPIO 都在 APB2 总线（Bus）上
// 再根据参考手册 7.3.7 节，APB2 的时钟启用寄存器 RCC_APB2ENR 偏移量为 0x18，得到其绝对地址为 0x40021018UL
#define RCC_ENABLE_PTR   (unsigned int *)0x40021018UL

int main(void)
{
  // 为了降低功耗，STM32 默认是关闭了所有外设的。现在我们要用 GPIO，就需要启用对应 Port C 的时钟
  // 根据参考手册 IO port C clock 是由 RCC_APB2ENR 的 4 号 bit （第 5 个）控制的，将它设为 1 即可启用时钟
  *RCC_ENABLE_PTR |= (1 << 4);

  // 还需要将 GPIO 13 端口配置为推挽（push-pull）输出模式，最高输出速率调为 50MHZ
  // 根据官方参考手册，为了实现上述配置，需要将 CNF13 设置为 00，将 MODE13 设为 01
  // MODE13 的起始 bit 号为 20，所以就是将 0b0011 左移 20 位，然后将值赋上去即可
  *GPIOC_CONFIG_PTR |= (0b0011 << 20);

  // 左移 13 位即得到 PC13 的 bit 位
  // 为了将此 bit 设为 0（低电平点亮），我们对地址取反再进行 & 运算
  *GPIOC_OUTPUT_PTR &= ~(1 << 13);

  while (1){};
}
```

使用 platformio 的 [Advanced] - [Verbose Build] 编译时将能观察到，它实际是使用了 `arm-none-eabi-gcc` 来编译的，流程如下：

1. 首先将所有依赖库的代码编译成 .o 文件
2. 然后将用户的 .c 代码编译成 .o 文件
3. 第三步，将 `$HOME/.platformio/packages/framework-stm32cubef1/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.S` 这个启动汇编文件编译输出为 `.pio/build/genericSTM32F103C8/FrameworkCMSISDevice/system_stm32f1xx.o`
4. 将前面启动汇编文件生成的 .o 文件，进一步转换为 `.pio/build/genericSTM32F103C8/libFrameworkCMSISDevice.a` 静态链接库
5. 最后，将前面生成的所有 .o 文件，以及 `startup_stm32f103xb.S` 汇编转换成的 .a 静态链接库链接到一起，生成出最后的固件 `.pio/build/genericSTM32F103C8/firmware.elf`

流程中的 `startup_stm32f103xb.S` 是开发板上电时最先启动的程序，可以理解成 bootloader，它先执行一些必备的动作，然后再调用我们 c 语言写的 main 函数。
详细内容可以直接查看该源码。

### 2. 第二个程序 - 还是点灯，但是更简单了


每次都自己计算寄存器地址与位偏移值太麻烦了，STM32 官方库通过结构体给我们定义了更方便的用法，我们可以利用官方库的定义将上述代码改成如下结构：

>这个结构体的定义原理如果有 C 语言基础的话很好理解，如果不好懂的话可以看看野火的教程，讲得非常细致。

```c
#include <stm32f103xb.h>

int main(void)
{
  // 为了降低功耗，STM32 默认是关闭了所有外设的。现在我们要用 GPIO，就需要启用对应 Port C 的时钟
  // 根据参考手册 IO port C clock 是由 RCC_APB2ENR 控制的，官方仓库直接提供了 RCC_APB2ENR_IOPCEN 用于设置这个 bit 位
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  // 还需要将 GPIO 13 端口配置为推挽（push-pull）输出模式，最高输出速率调为 50MHZ
  // 根据官方参考手册，为了实现上述配置，需要将 CNF13 设置为 00，将 MODE13 设为 01
  GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // 将 CNF13 与 MODE13 均设为 0b00
  GPIOC->CRH |= (GPIO_CRH_MODE13_0);    // 将 MODE13 的第 0 个比特位设为 1，即将 MODE13 设为 0b01

  // 根据官方参考手册 9.2.4 port output data register
  // 可以通过 ODR 来控制输出值
  // 将 GPIO13 对应的 bit 设为 0，输出低电平
  GPIOC->ODR = ~(1 << 13);

  while (1){};
}
```

最后我们借助 STM32 的 HAL_Delay 函数来实现 LED 灯延时闪烁：

>这里我也试过通过循环来实现软延时，但是不清楚是被编译器优化了还是怎样，没有任何效果...

```c
#include <stm32f1xx_hal.h>

int main(void)
{
  HAL_Init();

  // 为了降低功耗，STM32 默认是关闭了所有外设的。现在我们要用 GPIO，就需要启用对应 Port C 的时钟
  // 根据参考手册 IO port C clock 是由 RCC_APB2ENR 控制的，官方仓库直接提供了 RCC_APB2ENR_IOPCEN 用于设置这个 bit 位
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;

  // 还需要将 GPIO 13 端口配置为推挽（push-pull）输出模式，最高输出速率调为 50MHZ
  // 根据官方参考手册，为了实现上述配置，需要将 CNF13 设置为 00，将 MODE13 设为 01
  GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);  // 将 CNF13 与 MODE13 均设为 0b00
  GPIOC->CRH |= (GPIO_CRH_MODE13_0);    // 将 MODE13 的第 0 个比特位设为 1，即将 MODE13 设为 0b01

  while (1){
    // 根据官方参考手册 9.2.4 port output data register
    // 可以通过 ODR（Output Data Register 的缩写）来控制输出值
    // 将 GPIO13 对应的 bit 设为 0，输出低电平
    GPIOC->ODR = ~(1 << 13);
    HAL_Delay(500);
    // 将 GPIO13 对应的 bit 设为 1，输出高电平
    GPIOC->ODR |= (1 << 13);
    HAL_Delay(500);
  };
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}
```

### 3. 第三个程序 - 还是点灯，但是使用 HAL 库

前面的第二个版本仍然存在一些位操作，代码量多了后可读性仍然不是很好。

实际上 STM32 的 HAL 库做了更彻底的封装，将底层的位操作几乎完全隐藏起来了，使我们写代码就像是在搭积木一样调用各种封装好的函数就 OK 了。
这进一步提升了代码的可维护性。

使用 HAL 的点灯代码如下：

>这个野火的视频也做了细致的分析，一步步从原始寄存器操作优化到 HAL，非常清晰。我这里就跳过中间过程，直接开始使用 HAL 库了。

```c
// start of include/main.h
#ifndef MAIN_H
#define MAIN_H

#include "stm32f1xx_hal.h"

// LED 接的是 PC13，这里应该使用  GPIOC 这个 port
#define LED_GPIO_PORT             GPIOC
// 同样，也应该启用 GPIOC 的 Clock
#define LED_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()
// LED 所在的引脚为 PC13，对应 C Port 的 13 号引脚
#define LED_PIN                   GPIO_PIN_13


#endif
// end of include/main.h

// ==========================================

// start of src/main.c
#include "main.h"

void LED_Init();

int main(void)
{
  HAL_Init();
  LED_Init();

  while (1)
  {
    // 切换 LED 引脚的状态，原来是 1 就改成 0，原来是 0 就改成 1
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
    HAL_Delay(1000);
  }
}

void LED_Init()
{
  LED_GPIO_CLK_ENABLE(); // 启用 LED 的 GPIO 端口时钟

  GPIO_InitTypeDef GPIO_InitStruct = {
    Pin: LED_PIN,
    Mode: GPIO_MODE_OUTPUT_PP,
    Pull: GPIO_PULLUP,
    Speed: GPIO_SPEED_HIGH,
  };

  // 初始化 GPIO 对应的 port
  HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}
```


## 二、如何调试

在没有 DAP 调试器的情况下，解决方法是直接通过串口打印日志。

先回顾下我们之前的 platform.ini 配置内容，其中已经配置好了调试协议：

```ini
[env:learn-STM32F103C8]
platform = ststm32
board = genericSTM32F103C8
framework = stm32cube
# AVAILABLE upload_protocol: blackmagic, cmsis-dap, dfu, jlink, serial, stlink
upload_protocol = cmsis-dap
# 调试协议也设为 DAP
debug_tool = cmsis-dap
```

那么直接进入 VSCode 的 PlatformIO 插件页面，在右下角的「Debug」中直接点击「Start Debugging」就能开始调试了。
调试方法跟用 VSCode 调试一般程序也没啥区别，开始调试后会自动跳转到 VSCOde 的调试页面，左侧栏会显示各种变量的状态，也可以手动加 watch。

platformio 使用了 OpenOCD(Open On-Chip Debugger) + gdb 进行 STM32 的远程调试，因此在 debug console 中能输入各种 gdb 调试命令，用法可以参考下这个 [水一水GDB调试器的用法（入门+进阶）](https://0xffff.one/d/507-shui-yi-shui-gdb-tiao-shi-qi-de). 不过入门级的调试通过 UI 就能直接搞定了。

>这里因为我有 VSCode 与其他语言的调试经验，具体的调试方法就直接略过了。


### 反汇编

也可以直接使用 platformio 安装的工具链来反编译固件，通过其汇编查问题，不过需要懂一点 ARM 汇编：

```shell
~/.platformio/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-objdump -d .pio/build/genericSTM32F103C8/firmware.elf > .pio/firmware.asm
```


## 三、如何使用 STM32CubeMX + VSCode 写程序

STM32CubeMX 是 ST 官方提供的代码生成器，非常强大，能帮我们省很多事，野火教程也强烈推荐使用。

但是它生成出的项目结构跟 platformio 完全对不上，没办法直接在 platformio 上用。另一方面我仍然只想用 VSCode 写代码，不打算考虑 STM32CubeIDE/Keli 之类的商业软件或专有软件。


VSCode 上有这几种方法，可以编写、调试、上传、监控 STM32CubeMX 生成的项目：


1. platformio
2. Makefile 方案
   1. [stm32-for-vscode](https://github.com/bmd-studio/stm32-for-vscode): 要求在用 STM32CubeMX 生成代码时选择生成 Makefile
3. CMake 方案
   1. [cubemx.cmake](https://github.com/patrislav1/cubemx.cmake): 这个据说封装得比较好，比较小白
   2. [stm32-cmake](https://github.com/ObKo/stm32-cmake): 这个项目 stars 多，不过要求具有一定的 CMake 基础，看着有点复杂
   3. [stm32-cube-cmake-vscode](https://github.com/MaJerle/stm32-cube-cmake-vscode): 它要求安装 STM32CubeIDE，我只装了 STM32CubeMX，直接 Pass


上面这个列表整理自如下资料：

- [Develop STM32 with CubeMX and VSCode - Reddit](https://www.reddit.com/r/embedded/comments/urft51/develop_stm32_with_cubemx_and_vscode/)
- [using-stm32cubemx-and-platformio](https://community.platformio.org/t/using-stm32cubemx-and-platformio/2611/57)


下面我们分别过一下我尝试过的几种方案。

### 方案一 - 修改 platformio

我之前整 esphome/8051 都是用的 platformio，这个 vscode 插件知名度比较高，很容易上手。

但是它的源代码文件夹结构比较固定，与 STM32CubeMX 生成代码的结构差别较大，在与 stm32cubemx 结合时遇到了很多问题。

主要问题有：

1. platformio 源代码文件夹结构比较固定，与 STM32CubeMX 生成代码的结构差别较大
   1. 最简单的方法是将 stm32cubemx 生成的代码移动到 platformio 的 src 文件夹中，但这解决不了 stm32cube 底层依赖版本不一致的问题。（通常情况下可能并无此问题，所以这种做法是可行的）
2. platformio 官方的 stm32cube 版本也可能有落后，导致需要各种魔改 `platformio.ini` 去掉对 platformio 内部 stm32cube 的依赖，改用 SM32CubeMX 生成的 Drivers 替代。以及修改源代码位置，依赖项位置...这就会变得很麻烦
3. 第三个问题是：网上搜到的代码库也大都是 stm32cubeide 格式的，要利用上这些代码也需要将其移动到 platformio 的文件夹中，或者添加 library.json

最后找到了一个非常简单的方法使我通过 platformio 直接跑 STM32CubeMX 生成的程序，就是生成程序时，要选择生成带 Makefile 的项目，然后再手动写一个配置文件放在仓库根目录下，内容如下：

```ini
# 需要在 platformio 页面中搜到你的板子在 platformio 中的名称，替换掉这里的 genericSTM32F103C8
[env:genericSTM32F103C8]
platform = ststm32
board = genericSTM32F103C8
framework = stm32cube

# 如下几行为需要手动补充的内容：
## 使用 DAP 调试器上传固件
upload_protocol = cmsis-dap
## 调试协议也设为 DAP
debug_tool = cmsis-dap
# 通过 build_flags 来控制引入的依赖项
build_flags = -g
  -I MiddleWares/Xxx/
## platformio 正常情况下只能配一个源码文件夹，但是可以通过这个参数添加多个，而且路径是相对于 src_dir
## 详见：https://docs.platformio.org/en/stable/projectconf/sections/env/options/build/build_src_filter.html
# 这里设置多个是为了引入 STM32CubeMX 生成的 Drivers 依赖库
build_src_filter =
  +<*>
  -<../../.pio/>
  -<../../.vscode/>
  +<../../MiddleWares>


[platformio]
## 将默认代码文件夹设为 STM32CubeMX 生成的代码文件夹
include_dir=Core/Inc
src_dir=Core/Src
```

这样就 OK 了，其中主要的点是通过 `build_src_filter` `build_flags` 与 `include_dir` `src_dir` 来配置正确的源码查找位置。
但是这里并未排除掉官方 Drivers 源码，所以是直接使用的 platformio 官方的 stm32 Drivers，有时候可能会遇到版本不一致导致的兼容性问题，需要注意。

用这种方案的好处是 platformio 的编译、调试流程很方便顺手。

### 方案二 - 使用 CMake + VSCode 写代码

TODO

## 四、使用 STM32 连接显示器

>代码仓库：<4_stm32f103_ili9341>

我手上有这几块显示屏：

- LCD 液晶显示屏：0.96 寸，128 * 64，使用 I2C 协议，四个引脚，驱动 IC 为 SSD1315
- TFT SPI 显示屏
  - 2.8 寸，320 * 240，使用 4 数据线 SPI 协议，驱动 IC 为 ILI9341
  - 3.5 寸电阻触摸屏，480 * 320，同样是 SPI 协议，驱动 IC 为 ILI9488

首先尝试连接 320 * 240 的 SPI 显示屏，找到一个支持 STM32CubeMX 的显示器驱动仓库，我用到的 ILI9341/ILI9488 两种驱动它均支持：

- [stm32_hal_graphics_display_drivers](https://github.com/RobertoBenjami/stm32_hal_graphics_display_drivers)

我需要使用 SPI 协议连接显示器，查阅驱动仓库的 [Drivers/io_spi/lcd_io_spi_hal.h](https://github.com/RobertoBenjami/stm32_hal_graphics_display_drivers/blob/master/Drivers/io_spi/lcd_io_spi_hal.h) 注释可知，它需要我们在使用 STM32CubeMX 生成项目时，分别根据其注释设置 SPI/DMA/GPIO 三项参数，这样生成好的代码就能直接使用此仓库的驱动。

其他配置用多了就熟悉了，记得每次重新生成代码前都先 git commit 下，这样如果不小心删掉了你的代码，还能找回来。

生成好后，根据驱动项目的提示，需要将驱动中不需要的内容全部删除，然后将剩余内容添加到项目构建的导入路径中，比如说添加到 platformio 的 libs 文件夹中，注意要遵循 platformio 的 library 项目结构。

以 ILI9341 为例，修改完成后的项目存放在 `./stm32f103_ili9341` 中，项目 lib 中的结构为：

```shell
├── lib
│   ├── README
│   └── stm32_hal_graphics_display_drivers
│       ├── README.md
│       └── src
│           ├── bmp.h
│           ├── Fonts
│           │   ├── font12.c
│           │   ├── font16.c
│           │   ├── font20.c
│           │   ├── font24.c
│           │   ├── font8.c
│           │   ├── fonts.h
│           │   └── Release_Notes.html
│           ├── io_spi
│           │   ├── lcd_io_spi_hal.c
│           │   └── lcd_io_spi_hal.h
│           ├── lcd
│           │   ├── ili9341.c
│           │   └── ili9341.h
│           ├── lcd.h
│           ├── lcd_io.h
│           ├── stm32_adafruit_lcd.c
│           ├── stm32_adafruit_lcd.h
│           ├── stm32_adafruit_ts.c
│           ├── stm32_adafruit_ts.h
│           └── ts.h
```

其中删掉了 `lcd` 文件夹中 ili9341 之外的所有其他驱动文件，以及 io_spi 外的所有其他协议的文件夹，如果不删除掉它们，编译时就会报错重复的函数定义。

相关内容还可参考：[基于STM32的TFT-LCD各种显示实现（内容详尽含代码） - CSDN](https://blog.csdn.net/black_sneak/article/details/125583293)


