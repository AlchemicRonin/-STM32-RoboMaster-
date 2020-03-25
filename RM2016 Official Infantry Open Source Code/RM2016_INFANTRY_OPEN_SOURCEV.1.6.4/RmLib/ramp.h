#ifndef _RAMP__H_
#define _RAMP__H_
#include "main.h"

typedef struct RampGen_t
{
	int32_t count;
	int32_t XSCALE;
	float out;
	void (*Init)(struct RampGen_t *ramp, int32_t XSCALE);
	float (*Calc)(struct RampGen_t *ramp);
	void (*SetCounter)(struct RampGen_t *ramp, int32_t count);
	void (*ResetCounter)(struct RampGen_t *ramp);
	void (*SetScale)(struct RampGen_t *ramp, int32_t scale);
	uint8_t (*IsOverflow)(struct RampGen_t *ramp);
}RampGen_t;

#define RAMP_GEN_DAFAULT \
{ \
							.count = 0, \
							.XSCALE = 0, \
							.out = 0, \
							.Init = &RampInit, \
							.Calc = &RampCalc, \
							.SetCounter = &RampSetCounter, \
							.ResetCounter = &RampResetCounter, \
							.SetScale = &RampSetScale, \
							.IsOverflow = &RampIsOverflow, \
						} \

						
void RampInit(RampGen_t *ramp, int32_t XSCALE);
float RampCalc(RampGen_t *ramp);
void RampSetCounter(struct RampGen_t *ramp, int32_t count);
void RampResetCounter(struct RampGen_t *ramp);
void RampSetScale(struct RampGen_t *ramp, int32_t scale);
uint8_t RampIsOverflow(struct RampGen_t *ramp);

#endif

