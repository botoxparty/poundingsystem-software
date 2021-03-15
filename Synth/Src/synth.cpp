#include "synth.h"
#include "delay.h"

// Define the static Singleton pointer
Synth *Synth::inst_ = NULL;

float globaly;
uint16_t globalL;
float globalyL;

Synth *Synth::getInstance()
{
    if (inst_ == NULL)
    {
        inst_ = new Synth();
    }
    return (inst_);
}


Synth::Synth()
{
    static float f0 _CCM_;
    vol = env = 1;
    sound = SINE;
    autoFilterON = false;
    autoSound = 0;
    chorusON = false;
    delayON = true;
    phaserON = true;
    currentNote = 50;
    velocity = 127;
    frequency = 1000;
    Oscillator osc1;
    Oscillator osc2;
    Oscillator osc3;
    Oscillator osc4;
    Init();
}

void Synth::setCurrentNote(int note)
{
    currentNote = note;
}

void Synth::setFreq(int freq)
{
    frequency = freq;
}

void Synth::Init()
{
    Delay_init();
    DelayWet_set(127);
    DelayFeedback_set(120);
    // drifter_init();
    //	pitchGen_init();
    // sequencer_init();
    // ADSR_init(&adsr);
    // Chorus_init();
    // PhaserInit();
    // SVF_init();
    // filterFreq = 0.25f;
    // filterFreq2 = 0.25f;
    // osc_init(&op1, 0.8f, 587.f);
    // osc_init(&op2, 0.8f, 587.f);
    // osc_init(&op3, 0.8f, 587.f);
    // osc_init(&op4, 0.8f, 587.f);
    // osc_init(&vibr_lfo, 0, VIBRATO_FREQ);
    // osc_init(&filt_lfo, 0, 0);
    // osc_init(&filt2_lfo, 0, 0);
    // osc_init(&amp_lfo, 0, 0);
    // AdditiveGen_newWaveform();
    // VCO_blepsaw_Init(&mbSawOsc);
    // VCO_bleprect_Init(&mbRectOsc);
    // VCO_bleptri_Init(&mbTriOsc);
}

void Synth::make_sound(uint16_t *buf, uint16_t length) // To be used with the Sequencer
{
    uint16_t pos;
    uint16_t *outp;
    float y = 0;
    float yL, yR;
    float f1;
    uint16_t valueL, valueR;

    outp = buf;

    for (pos = 0; pos < length; pos++)
    {
        /*--- Sequencer actions and update ---*/
        //sequencer_process(); //computes f0 and calls sequencer_newStep_action() and sequencer_newSequence_action()
        // GRA
        // if (sequencerIsOn == true) {
        // 	sequencer_process(); //computes f0 and calls sequencer_newStep_action() and sequencer_newSequence_action()
        // } else {
        // f0 = notesFreq[currentNote];
        f0 = frequency;
        vol = (float)velocity / 127.0f;
        // }

        /*--- compute vibrato modulation ---*/
        // f1 = f0 * (1 + Osc_WT_SINE_SampleCompute(&vibr_lfo));

        /*--- Generate waveform ---*/
        y = generate_waveform(f0);
        globaly = y;

        /*--- Apply envelop and tremolo ---*/
        // env = ADSR_computeSample(&adsr) * (1 + Osc_WT_SINE_SampleCompute(&amp_lfo));
        // y *= vol * env; // apply volume and envelop
        y *= vol; // apply volume and envelop
        //if (adsr.cnt_ >= seq.gateTime)			ADSR_keyOff(&adsr);

        // GRA

        // if (sequencerIsOn == true)
        // {
        //     if (adsr.cnt_ >= seq.gateTime)
        //         ADSR_keyOff(&adsr);
        // }

        /*--- Apply filter effect ---*/
        /* Update the filters cutoff frequencies */
        // if ((!autoFilterON) && (filt_lfo.amp != 0))
        //     SVF_directSetFilterValue(&SVFilter,
        //                              filterFreq * (1 + OpSampleCompute7bis(&filt_lfo)));
        // if (filt2_lfo.amp != 0)
        //     SVF_directSetFilterValue(&SVFilter2,
        //                              filterFreq2 * (1 + OpSampleCompute7bis(&filt2_lfo)));
        // y =
        //     0.5f * (SVF_calcSample(&SVFilter, y) + SVF_calcSample(&SVFilter2, y)); // Two filters in parallel

        /*---  Apply delay effect ----*/
        if (delayON)
            y = Delay_compute(y);

        // /*---  Apply phaser effect ----*/
        // if (phaserON)
        //     y = Phaser_compute(y);

        // /*--- Apply chorus/flanger effect ---*/
        // if (chorusON)
        //     stereoChorus_compute(&yL, &yR, y);
        // else
        //     yL = yR = y;

        /*--- clipping ---*/

        yL = yR = y;

        yL = (yL > 1.0f) ? 1.0f : yL; //clip too loud left samples
        yL = (yL < -1.0f) ? -1.0f : yL;

        yR = (yR > 1.0f) ? 1.0f : yR; //clip too loud right samples
        yR = (yR < -1.0f) ? -1.0f : yR;

        /****** let's hear the new sample *******/

        valueL = (uint16_t)((int16_t)((32767.0f) * yL)); // conversion float -> int
        valueR = (uint16_t)((int16_t)((32767.0f) * yR));

        *outp++ = valueL; // left channel sample
        *outp++ = valueR; // right channel sample
    }
}

float Synth::generate_waveform(float freq)
{
    float y;

    // Set osc1 frequency
    osc1.freq = freq;

    switch (sound)
    {
    case SINE:
        y = SoundGenerator::FM(&osc4, &osc3, &osc2, &osc1);
        break;
    default:
        y = SoundGenerator::SineTable(&osc1);
        break;
    }

    // /* choose waveform generator */
    // switch (sound)
    // {
    // case MORPH_SAW:
    //     y = 0.8f * MorphingSaw_SampleCompute(&op1);
    //     break;

    // case SPLIT:
    // {
    //     // if (frq < 200)
    //     //     y = OpSampleCompute1(&op1);
    //     // else if (frq < 600)
    //     //     y = OpSampleCompute2(&op1);
    //     // else
    //     //     y = OpSampleCompute5(&op1);
    // }
    // break;

    // case ACC_SINE:
    //     y = 0.8 * OpSampleCompute0(&op1);
    //     break;

    // case POWER_SINE:
    //     y = OpSampleCompute3(&op1);
    //     break;

    // case BLEPTRIANGLE:
    //     mbTriOsc.freq = frq;
    //     y = VCO_bleptri_SampleCompute(&mbTriOsc);
    //     break;

    // case BLEPSQUARE:
    //     mbRectOsc.freq = frq;
    //     y = VCO_bleprect_SampleCompute(&mbRectOsc);
    //     break;

    // case WT_SINE:
    //     y = 0.8f * Osc_WT_SINE_SampleCompute(&op1);
    //     break;

    // case ADDITIVE:
    //     y = AdditiveGen_SampleCompute(&op1);
    //     break;

    // case NOISE:
    //     y = op1.amp * frand_a_b(-.8f, .8f);
    //     break; // noise !

    // case CHORD15:
    // { // fundamental + fifth : 1 5
    //     OpSetFreq(&op2, frq * 1.50f);
    //     y = 0.5f * (MorphingSaw_SampleCompute(&op1) + MorphingSaw_SampleCompute(&op2));
    // }
    // break;

    // case CHORD135:
    // { // major chord : 1 3maj 5
    //     OpSetFreq(&op2, frq * 1.26f);
    //     OpSetFreq(&op3, frq * 1.5f);
    //     y = 0.33f * (MorphingSaw_SampleCompute(&op1) + MorphingSaw_SampleCompute(&op2) + MorphingSaw_SampleCompute(&op3));
    // }
    // break;

    // case CHORD13min5:
    // { // minor chord : 1 3min 5
    //     OpSetFreq(&op2, frq * 1.1892f);
    //     OpSetFreq(&op3, frq * 1.5f);
    //     y = 0.33f * (MorphingSaw_SampleCompute(&op1) + MorphingSaw_SampleCompute(&op2) + MorphingSaw_SampleCompute(&op3));
    // }
    // break;

    // case VOICES3:
    // { // 3 slightly detuned oscillators with drifters

    //     OpSetFreq(&op2, frq * (1 + d1_drifter_nextSample()));
    //     OpSetFreq(&op3, frq * (1 + d2_drifter_nextSample()));
    //     y = 0.33f * (MorphingSaw_SampleCompute(&op1) + MorphingSaw_SampleCompute(&op2) + MorphingSaw_SampleCompute(&op3));
    // }
    // break;

    // case DRIFTERS:
    //     y = Drifters_sampleCompute();
    //     break;

    // case FM2:
    //     y = FM2_sampleCompute(frq);
    //     break;

    // case BLEPSAW:
    // {
    //     mbSawOsc.freq = frq;
    //     y = VCO_blepsaw_SampleCompute(&mbSawOsc);
    // }
    // break;

    // default:
    //     y = 0;
    //     break;
    // }

    return osc1.out;
}
