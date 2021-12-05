/**
  ******************************************************************************
  * @file	main.c
  * @author  Ac6
  * @version V1.0
  * @date	01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdint.h>
#include "midi.h"

//#include "maple-leaf-rag.c"
#include <string.h> // for memset() declaration
#include <math.h>   // for MA_PI






void init_TIM2(int tick){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
   	TIM2->CR1 &= ~TIM_CR1_CEN;
   	TIM2->CR2 |= TIM_CR2_MMS_1;
   	TIM2->PSC = 48-1;
   	TIM2->ARR = tick-1;
   	TIM2->DIER |= TIM_DIER_UIE;
   	TIM2->CR1 |= TIM_CR1_CEN;
   	NVIC_EnableIRQ(TIM2_IRQn);
}

void init_TIM6(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->CR1 &= ~TIM_CR1_CEN;
    TIM6->CR2 |= TIM_CR2_MMS_1;
    TIM6->PSC = 10-1;
    TIM6->ARR = 160-1;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] |= 1 << TIM6_DAC_IRQn;
}

struct {
	uint8_t note;
	uint8_t chan;
	uint8_t volume;
	int 	step;
	int 	offset;
} voice[15];

#define N 1000
#define RATE 20000
short int wavetable[N];
//int step = 0;
//int offset = 0;




void TIM6_DAC_IRQHandler(void)
{
	// TODO: Remember to acknowledge the interrupt right here!
	TIM6->SR &= ~TIM_SR_UIF;



	int sample = 0;
//	sample = wavetable[155];

    	for(int i=0; i < sizeof voice / sizeof voice[0]; i++) {
        	sample += (wavetable[voice[i].offset>>16] * voice[i].volume) /*<< 4*/;
        	voice[i].offset += voice[i].step;
        	if ((voice[i].offset >> 16) >= sizeof wavetable / sizeof wavetable[0])
            	voice[i].offset -= (sizeof wavetable / sizeof wavetable[0]) << 16;
    	}
    	sample = (sample >> 16) + 2048;
    	DAC->DHR12R1 = sample;
}

void TIM2_IRQHandler(void)
{
	// TODO: Remember to acknowledge the interrupt right here!
	TIM2->SR &= ~TIM_SR_UIF;

	midi_play();
}

void set_tempo(int time, int value, const MIDI_Header *hdr)
{
	// This assumes that the TIM2 prescaler divides by 48.
	// It sets the timer to produce an interrupt every N
	// microseconds, where N is the new tempo (value) divided by
	// the number of divisions per beat specified in the MIDI header.
	TIM2->ARR = value/hdr->divisions - 1;
}



void init_DAC(){
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
   	GPIOA->MODER |= 3<<(2*4);
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	DAC->CR &= ~DAC_CR_EN1;
	DAC->CR |= DAC_CR_TEN1;
	DAC->CR |= DAC_CR_TSEL1;
	DAC->CR |= DAC_CR_EN1;
}


void note_on(int time, int chan, int key, int velo)
	{
  	for(int i=0; i < sizeof voice / sizeof voice[0]; i++)
    	if (voice[i].step == 0) {
      	// configure this voice to have the right step and volume
        	voice[i].step = time;
        	voice[i].note = key;
        	voice[i].volume = velo;
      	break;
    	}
	}

void note_off(int time, int chan, int key, int velo)
{
  for(int i=0; i < sizeof voice / sizeof voice[0]; i++)
	if (voice[i].step != 0 && voice[i].note == key) {
  	// turn off this voice
  	voice[i].step = 0;
  	voice[i].note = 0;
  	voice[i].volume = 0;
  	break;
	}
}



extern uint8_t midifile[];
int main(void)
{
    for(int i=0; i < N; i++)
            wavetable[i] = 32767 * sin(2 * M_PI * i / N);

	init_DAC();

	MIDI_Player *mp = midi_init(midifile);
	// The default rate for a MIDI file is 2 beats per second
	// with 48 ticks per beat.  That's 500000/48 microseconds.
	init_TIM2(10417);
	init_TIM6();
	// Nothing else to do at this point.
	for(;;){
	    if (mp->nexttick >= MAXTICKS)
	            midi_init(midifile);
    	asm("wfi");
    	}


}



