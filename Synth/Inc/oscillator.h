
#ifndef _PS_OSC_
#define _PS_OSC_
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/*-------------------------------------------------------*/
typedef struct
{
	float_t 	amp;	// should be <= 1 for normal sound output
	float_t		last_amp;
	float_t 	freq;	// Hertz
	float_t 	phase;	// radians
	float_t		phi0;	// radians
	float_t		modInd;	// Modulation Index for FM
	float_t		mul;	// pitch frequency multiplier
	float_t 	out;	// output sample

} Osc;
/*-------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

enum timbre { MORPH_SAW = 0, SPLIT, ACC_SINE, WT_SINE, ADDITIVE, POWER_SINE, BLEPTRIANGLE, BLEPSQUARE,
	NOISE, CHORD15, CHORD135, CHORD13min5, VOICES3, DRIFTERS, FM2, BLEPSAW, LAST_SOUND };
    
#ifdef __cplusplus
class Oscillator
{
public:
    Oscillator(float_t 	a = 1, float_t f = 500);
	float_t 	amp;	// should be <= 1 for normal sound output
	float_t		last_amp;
	float_t 	freq;	// Hertz
	float_t 	phase;	// radians
	float_t		phi0;	// radians
	float_t		modInd;	// Modulation Index for FM
	float_t		mul;	// pitch frequency multiplier
	float_t 	out;	// output sample

    private:
    void Init();
};
#endif
#endif