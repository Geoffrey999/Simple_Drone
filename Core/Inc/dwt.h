#ifndef	__DWT_H
#define	__DWT_H

#include "main.h"

void DWT_Init(void);
uint32_t get_DWT_count(void);
uint32_t DWT_GetMS(void);
uint32_t DWT_GetUS(void);
#endif
