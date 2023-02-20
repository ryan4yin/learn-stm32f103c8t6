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
    // 可以通过 ODR 来控制输出值
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