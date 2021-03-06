
//Serial code is common for all the cortex M cores, so it has been put here

#ifdef _ARCH_ARM7_LPC2000
#include "serial_lpc2000.h"
#elif defined(_ARCH_CORTEXM3_STM32)   || defined(_ARCH_CORTEXM4_STM32F4) \
   || defined(_ARCH_CORTEXM3_STM32F2) || defined(_ARCH_CORTEXM3_STM32L1)
#include "serial_stm32.h"
#elif defined(_ARCH_CORTEXM3_EFM32GG)
#include "serial_efm32.h"
#else
#error "Unknown arch"
#endif
