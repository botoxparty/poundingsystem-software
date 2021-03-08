#include "oscillator.h"

Oscillator::Oscillator(float_t a, float_t f)
{
    amp = a;
    last_amp = a;
    freq = f;
    Init();
};

void Oscillator::Init()
{
    phase = 0;
    out = 0;
    modInd = 0;
    mul = 1;
}