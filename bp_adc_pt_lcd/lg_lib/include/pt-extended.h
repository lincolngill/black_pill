/*
 * pt-extended.h - Extended Protothread library - For stm32f103c8
 */

#ifndef __PT_EXTENDED
#define __PT_EXTENDED

#ifdef __cplusplus
 extern "C" {
#endif

#include "pt.h"                         // Original Protothread library
#include "pt-sem.h"                     // Original Protothread semaphore library


// Yield for a set delay in ms.
// pt_msTicks must be incremented by a timer ISR. E.g. SysTick
extern volatile unsigned int pt_msTicks;
#define PT_YIELD_TIME_msec(pt, delay_time) \
   do { static unsigned int time_thread; \
   time_thread = pt_msTicks + (unsigned int)delay_time; \
   PT_YIELD_UNTIL(pt, pt_msTicks >= time_thread); \
   } while(0);

#define PT_GET_TIME() (pt_msTicks)

void PT_SETUP(void);
void SysTick_Handler(void);

#define PT_SEM_SET(s) (s)->count=1
#define PT_SEM_CLEAR(s) (s)->count=0
#define PT_SEM_READ(s) (s)->count
#define PT_SEM_ACCEPT(s) \
  s->count; \
  if (s->count) s->count-- ; \

#ifdef __cplusplus
}
#endif

#endif // _PT_EXTENDED
