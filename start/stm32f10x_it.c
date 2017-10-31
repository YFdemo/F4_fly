
#include "stm32f10x_it.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  /*进入无限循环，当硬件故障发生异常*/ 
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  /*进入无限循环，当内存管理出现异常*/ 
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /*进入无限循环，当总线故障异常发生*/
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /*进入无限循环，当使用故障发生异常*/ 
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
}
