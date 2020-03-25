#ifndef RNG_H
#define RNG_H
#include "main.h"

extern uint8_t RNG_init(void);
extern unsigned int RNG_get_random_num(void);
extern int RNG_get_random_range(int min, int max);
#endif
