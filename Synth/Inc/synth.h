
#ifndef _PS_SYNTH_
#define _PS_SYNTH_
#include "oscillator.h"
#include "soundgenerator.h"
#include "notesTable.h"
#include "audiocodec.h"

#define _CCM_ __attribute__((section(".ccmram"))) // for use of CCM RAM (64kB)

extern uint16_t audiobuff[BUFF_LEN];

/*-------------------------------------------------------*/
#define MAX_FILTER_LFO_AMP 32.f
#define DECAY_FACTOR 0.99975f // initial multiplying factor for the envelope decay
#define MIN_DECAY_FACTOR 0.999f
#define VIBRATO_AMP 0.01f
#define VIBRATO_FREQ 5.5f
#define MAX_VIBRATO_AMP 0.5f
#define MAX_VIBRATO_FREQ 12.f
/*-------------------------------------------------------*/
#ifdef __cplusplus

class Synth
{
public:
    static Synth *getInstance();
    Synth();
    bool autoFilterON;
    bool delayON;
    bool phaserON;
    bool chorusON;
    int8_t autoSound;
    float f0;
    float vol;
    float env;
    int currentNote;
    int velocity;
    int frequency;
    Sounds sound;
    float generate_waveform(float freq);
    void make_sound(uint16_t *buf, uint16_t length);
    void setCurrentNote(int note);
    void setFreq(int freq);
    Oscillator osc1;
    Oscillator osc2;
    Oscillator osc3;
    Oscillator osc4;
private:
    void Init();
    static Synth *inst_;   // The one, single instance
    Synth(const Synth &);
    Synth &operator=(const Synth &);
};

#endif
#endif
