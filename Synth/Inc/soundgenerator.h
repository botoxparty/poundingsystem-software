
#ifndef _PS_SOUND_GENERATOR_
#define _PS_SOUND_GENERATOR_

#include "oscillator.h"
#include "sinetable.h"

#define _2PI 6.283185307f
#define _PI 3.14159265f
#define SAMPLERATE 44100
//#define Fs             			(float)SAMPLERATE	/* samplerate */
#define Ts (1.f / SAMPLERATE) // sample period

#ifdef __cplusplus

enum Sounds
{
    SINE,
    SINE2,
    SAWTOOTH,
    SQUARE,
    SAXXX,
    JAPPY,
    STAR,
    MICROWAVE,
};

class SoundGenerator
{
public:
    SoundGenerator();
    static const int soundscount = 8;
    static char* soundNames[8];
    static float Sine(Oscillator *osc);
    static float PSSine(Oscillator *osc);
    static float_t SineTable(Oscillator *op);
    static float_t FM(Oscillator *osc4, Oscillator *osc3, Oscillator *osc2, Oscillator *osc1);
    static float FM_sine_SampleCompute(Oscillator * op, float FMinput);
private:
};

#endif
#endif
