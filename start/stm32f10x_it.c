
#include "stm32f10x_it.h"

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  /*��������ѭ������Ӳ�����Ϸ����쳣*/ 
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  /*��������ѭ�������ڴ��������쳣*/ 
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  /*��������ѭ���������߹����쳣����*/
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  /*��������ѭ������ʹ�ù��Ϸ����쳣*/ 
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
