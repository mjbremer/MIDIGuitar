#ifndef _PROTOTYPE_H_
#define _PROTOTYPE_H_

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include "kiss_fft.h"

//#ifndef PITCH_DETECT_LIB_DEBUG
//#include "main.h"
//#endif

#define FS (48828U)
#define FS_FLOAT (48828.0f)
#define MAX_FREQ_GUITAR (1200.0f)
#define MIN_FREQ_GUITAR (80.0f)

#define MIN_PERIOD_GUITAR (1.0f / MAX_FREQ_GUITAR)
#define MAX_PERIOD_GUITAR (1.0f / MIN_FREQ_GUITAR)




#define MAJOR_SCALE_LEN (12U)
#define NUM_MIDI_NOTES (128U)

#define FRAME_SIZE (1250U)
#define ZP_FACTOR (1U)
#define FFT_SIZE (FRAME_SIZE * ZP_FACTOR)

extern double ceps[FFT_SIZE];

int init_vars(void); //initializing variables
int copy_major_into_midi_enable(int n);
int apply_major_scale(int n);
int argmax(double* l, size_t len);
int quantize_frequency_to_MIDI(float in_freq);
const char* note_name_from_midi_num(int n);
int detect_pitch(int32_t*buf);
int main();

//put declares in H boi
#endif // _PROTOTYPE_H_
