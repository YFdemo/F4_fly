#include "sys.h"

extern u32 pwmout1;	//捕获总高电平的时间
extern u32 pwmout2;	//捕获总高电平的时间
extern u32 pwmout3;	//捕获总高电平的时间
extern u32 pwmout4;	//捕获总高电平的时间

void TIM4_Cap_Init(u16 arr, u16 psc);
