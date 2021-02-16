#include "prototype.h" //quotes look in this file directory
//#define PITCH_DETECT_LIB_DEBUG 0

float t [FRAME_SIZE]; //un-initialized array
size_t min_index, max_index;
kiss_fft_cfg first;
kiss_fft_cfg second;
double ceps[FFT_SIZE]; // Only global in case we need to dump over UART from main()
//struct kiss_fft_state first_s, second_s;

uint8_t midi_enable [NUM_MIDI_NOTES];

const char* const midi_note_names[] = {"LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","LOW","A0","A#0/Bb0","B0","C1","C#1/Db1","D1","D#1/Eb1","E1","F1","F#1/Gb1","G1","G#1/Ab1","A1","A#1/Bb1","B1","C2","C#2/Db2","D2","D#2/Eb2","E2","F2","F#2/Gb2","G2","G#2/Ab2","A2","A#2/Bb2","B2","C3","C#3/Db3","D3","D#3/Eb3","E3","F3","F#3/Gb3","G3","G#3/Ab3","A3","A#3/Bb3","B3","C4 (middle C)","C#4/Db4","D4","D#4/Eb4","E4","F4","F#4/Gb4","G4","G#4/Ab4","A4 concert pitch","A#4/Bb4","B4","C5","C#5/Db5","D5","D#5/Eb5","E5","F5","F#5/Gb5","G5","G#5/Ab5","A5","A#5/Bb5","B5","C6","C#6/Db6","D6","D#6/Eb6","E6","F6","F#6/Gb6","G6","G#6/Ab6","A6","A#6/Bb6","B6","C7","C#7/Db7","D7","D#7/Eb7","E7","F7","F#7/Gb7","G7","G#7/Ab7","A7","A#7/Bb7","B7","C8","C#8/Db8","D8","D#8/Eb8","E8","F8","F#8/Gb8","G8","G#8/Ab8","A8","A#8/Bb8","B8","C9","C#9/Db9","D9","D#9/Eb9","E9","F9","F#9/Gb9","G9"};
const uint8_t major_scale[]  = {1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0, 1};

int init_vars(void){
    for (int i = 0; i < FRAME_SIZE; i++) {
        t[i] = ((float)(i) / FS_FLOAT);
    }
    for (int i = 0; i < NUM_MIDI_NOTES; i++) {
        midi_enable[i] = 1;
    }
    min_index = ceilf(MIN_PERIOD_GUITAR * FS);
    max_index = floorf(MAX_PERIOD_GUITAR * FS);

    first = kiss_fft_alloc(FFT_SIZE, 0, NULL , NULL);
    if (first == NULL) {
		#ifdef PITCH_DETECT_LIB_DEBUG
    	printf("Unable to allocate space for KissFFT config\n");
		#endif
    	return -1;
    }

    second = kiss_fft_alloc(FFT_SIZE, 1, NULL , NULL);

    if (second == NULL)
    	return -1;

    return 0;

} 

int copy_major_into_midi_enable(int n){
    for (int i = 0; i< MAJOR_SCALE_LEN; i++) {
        if (n + i >= NUM_MIDI_NOTES){
            break;
        }
        else if (n + i >= 0){
            midi_enable[n+i] = major_scale[i];
        }
    }
    return 0;
}

int apply_major_scale(int n){

	if (n < 0 || n > 11)
		return -1;

    for (int i =0; i< 1 + ceilf((116.0f - n)/12.0f); i++){
        printf("%d\n", i);
        copy_major_into_midi_enable(n + (12*i));
    }
    return 0;
}    

// returns the index of the greatest value in the list
int argmax(double* l, size_t len){
    if (len == 0 || l == NULL) return -1; //good error checking

    double maximum = l[0];
    int maxindex = 0;
    for (int i = 0; i < len; i++){
        if (l[i] > maximum){
            maximum = l[i];
            maxindex = i;
        } 
    }
    return maxindex;
}

int quantize_frequency_to_MIDI(float in_freq){
    float n = 0;
    n = in_freq;
    n = n / 440.0f;
    n = log2f(n);
    n = n * 12;
    n = n + 69;
return roundf(n);
}

const char* note_name_from_midi_num(int n){
    return midi_note_names [n];
}

//int do_fft(kiss_fft_cpx* in, kiss_fft_cpx* out, size_t size, int is_ifft)
//{
//
//    kiss_fft((is_ifft ? second : first), in, out);
//
//return 0;
//}

int detect_pitch(int32_t*buf)
{
    size_t ceps_peak;
    float guess_freq;
    int guess_midi;
    //float ceps[FFT_SIZE];

    kiss_fft_cpx Fin[FFT_SIZE];
    kiss_fft_cpx Fout[FFT_SIZE];
    for (int i = 0; i < FFT_SIZE; i++) {
        Fin[i].r = (float) (buf[i]);
        Fin[i].i = 0;
        // printf("%f\n", Fin[i].r);
    }


    //do_fft(Fin, Fout, FFT_SIZE, 0); //set up kiss_fft buffers
    kiss_fft(first, Fin, Fout);

    for (int i = 0; i < FFT_SIZE; i++) {
        //Fin[i].r = logf(powf(Fout[i].r, 2) + (powf(Fout[i].i, 2)));
    	Fin[i].r = powf(Fout[i].r, 2) + (powf(Fout[i].i, 2));
        Fin[i].i = 0;
        //printf("%f, %f\n", Fout[i].r, Fout[i].i);
    }

    //do_fft(Fin, Fout, FRAME_SIZE, 1); //1 bc iFFT
    kiss_fft(second, Fin,Fout);


    for (int i = 0; i < FFT_SIZE; i++) {

        //printf("%f, %f\n", Fout[i].r, Fout[i].i);
        ceps[i] = pow(Fout[i].r, 2) + pow(Fout[i].i, 2);
    }
    // Cepstrum done, now find peak

    ceps_peak = argmax(&ceps[min_index], max_index - min_index) + min_index;
    guess_freq = 1.0f / t[ceps_peak];
    guess_midi = quantize_frequency_to_MIDI(guess_freq);

    //for (int i = min_index; i <= max_index; i++)
    //  printf("%f\n", ceps[i]);  

    //printf("%zu\n", ceps_peak);
    //printf("%f\n", guess_freq);
    //printf("%d\n", guess_midi);



    return guess_midi;
}

#ifdef PITCH_DETECT_LIB_DEBUG
#define NUM_BUFFERS 1

int main()
{
    //FILE* f_out;
    FILE* f_in;
    
    //int32_t testdata[FRAME_SIZE];
    int32_t testdata[NUM_BUFFERS * FRAME_SIZE];
    int guess_midi;
 
    if (init_vars() != 0) {
    	printf("Failed to Initialize Variables. Exiting...\n");
    	return 1;
    }
    f_in = fopen("signal.dat", "rb");

    if (f_in == NULL) {
    	printf("Failed to open signal.dat. Exiting...\n");
    	return 1;
    }

    if (fread(testdata, sizeof(int32_t), FRAME_SIZE*NUM_BUFFERS, f_in) != FRAME_SIZE*NUM_BUFFERS) {
    	printf("Failed to read %u items of size %zu from signal.dat. Exiting...\n", FRAME_SIZE*NUM_BUFFERS, sizeof(int32_t));
    	return 1;
    }
    //fread(testdata, sizeof(int32_t), 1000 * FRAME_SIZE, f_in);
    fclose(f_in);

    printf("%d\n", testdata[0]);


//    guess_midi = detect_pitch(testdata);
//    printf("%d\n", guess_midi);
    for (int i = 0; i < NUM_BUFFERS * FRAME_SIZE; i += FRAME_SIZE) {
        guess_midi = detect_pitch(&testdata[i]);
        printf("%d\n", guess_midi);
    }

    // // Write cepstrum to file
    // f_out = fopen("fft.dat", "w");
    // for (i = min_index; i < max_index +1; i++) {
    //     outdata[i] = pow(Fout[i].r, 2) + pow(Fout[i].i, 2); 
    //     fprintf(f_out, "%f\n", outdata[i]);
    // }
    // fclose(f_out);
    return 0;
}
#endif
