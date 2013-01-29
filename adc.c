/** \file adc.c
  *
  * \brief Driver for the PIC32's analog-to-digital converter (ADC).
  *
  * Analog-to-digital conversions are initiated by Timer3, so that the rate
  * of conversions is about 22.05 kHz. This sample rate was chosen because
  * it's a "standard" audio sample rate, so most audio programs can handle
  * PCM data at that rate. It's slow enough that the code in fft.c can handle
  * real-time FFTs at that sample rate. Conversions are done with a fixed
  * period in between each conversion so that the results of FFTs are
  * meaningful.
  *
  * The results of conversions go into #adc_sample_buffer. To begin a series
  * of conversions, call beginFillingADCBuffer(), then wait
  * until #sample_buffer_full is non-zero. #adc_sample_buffer will then
  * contain #SAMPLE_BUFFER_SIZE samples. This interface allows one buffer of
  * samples to be collected while the previous one is processed, which speeds
  * up entropy collection.
  *
  * For details on hardware interfacing requirements, see initADC().
  *
  * All references to the "PIC32 Family Reference Manual" refer to section 17,
  * revision E, obtained from
  * http://ww1.microchip.com/downloads/en/DeviceDoc/61104E.pdf
  * on 28 January 2013.
  *
  * This file is licensed as described by the file LICENCE.
  */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <p32xxxx.h>
#include "adc.h"
#include "pic32_system.h"
#include "ssd1306.h"

/** A place to store samples from the ADC. When #sample_buffer_full is
  * non-zero, every entry in this array will be filled with ADC samples
  * taken periodically. */
volatile uint16_t adc_sample_buffer[SAMPLE_BUFFER_SIZE];
/** Index into #sample_buffer where the next sample will be written. */
static volatile uint32_t sample_buffer_current_index;
/** This will be zero if #sample_buffer is not full. This will be non-zero
  * if #sample_buffer is full. */
volatile int sample_buffer_full;

/** Set up the PIC32 ADC to sample from AN2 periodically using Timer3 as the
  * trigger. */
void initADC(void)
{
	AD1CON1bits.ON = 0; // turn ADC module off
	asm("nop"); // just to be safe
	// This follows section 17.4 of the PIC32 family reference manual.
	AD1PCFGbits.PCFG2 = 0; // set AN2 pin to analog mode
	TRISBbits.TRISB2 = 1; // set RB2 as input (disable digital output)
	TRISCbits.TRISC13 = 1; // set RC13 as input (disable digital output)
	TRISCbits.TRISC14 = 1; // set RC14 as input (disable digital output)
	AD1CHSbits.CH0SA = 2; // select AN2 as MUX A positive source
	AD1CHSbits.CH0NA = 0; // select AVss as MUX a negative source
	AD1CON1bits.FORM = 4; // output format = 32 bit integer
	AD1CON1bits.SSRC = 2; // use Timer3 to trigger conversions
	AD1CON1bits.ASAM = 1; // enable automatic sampling
	AD1CON2bits.VCFG = 0; // use AVdd/AVss as references
	AD1CON2bits.CSCNA = 0; // disable scan mode
	AD1CON2bits.SMPI = 7; // 8 samples per interrupt
	AD1CON2bits.BUFM = 1; // double buffer mode
	AD1CON2bits.ALTS = 0; // disable alternate mode (always use MUX A)
	AD1CON3bits.ADRC = 0; // derive ADC conversion clock from PBCLK
	// Don't need to set SAMC since ADC is not in auto-convert (continuous)
	// mode.
	AD1CON3bits.SAMC = 12; // sample time = 12 ADC conversion clocks
	AD1CON3bits.ADCS = 14; // ADC conversion clock = 1.2 MHz
	AD1CON1bits.SIDL = 1; // discontinue operation in idle mode
	AD1CON1bits.CLRASAM = 0; // don't clear ASAM; overwrite buffer contents
	AD1CON1bits.SAMP = 0; // don't start sampling immediately
	AD1CON2bits.OFFCAL = 0; // disable offset calibration mode
	AD1CON1bits.ON = 1; // turn ADC module on
	IPC6bits.AD1IP = 3; // priority level = 3
	IPC6bits.AD1IS = 0; // sub-priority level = 0
	IFS1bits.AD1IF = 0; // clear interrupt flag
	IEC1bits.AD1IE = 1; // enable interrupt
	delayCycles(144); // wait 4 microsecond for ADC to stabilise
	// Initialise Timer3 to trigger ADC conversions.
	T3CONbits.ON = 0; // turn timer off
	T3CONbits.SIDL = 0; // continue operation in idle mode
	T3CONbits.TCKPS = 0; // 1:1 prescaler
	T3CONbits.TGATE = 0; // disable gated time accumulation
	T3CONbits.SIDL = 0; // continue in idle mode
	TMR3 = 0; // clear count
	PR3 = 1633; // frequency = about 22045 Hz
	IFS0bits.T3IF = 0; // clear interrupt flag
	IEC0bits.T3IE = 0; // disable timer interrupt
}

/** Insert new sample into ADC sample buffer (#adc_sample_buffer).
  * \param sample The sample to insert.
  */
static void insertSample(uint32_t sample)
{
	sample &= 0x3ff;
	if (sample_buffer_current_index >= SAMPLE_BUFFER_SIZE)
	{
		T3CONbits.ON = 0; // turn timer off
		sample_buffer_full = 1;
	}
	else
	{
		adc_sample_buffer[sample_buffer_current_index] = (uint16_t)sample;
		sample_buffer_current_index++;
	}
}

/** Interrupt handler that is called whenever an analog-to-digital conversion
  * is complete. The priority level is set to 3 so that this can interrupt
  * USB interrupts. */
void __attribute__((vector(_ADC_VECTOR), interrupt(ipl3), nomips16)) _ADCHandler(void)
{
	if (AD1CON2bits.BUFS == 0)
	{
		// ADC is currently filling buffers 0 - 7.
		insertSample(ADC1BUF8);
		insertSample(ADC1BUF9);
		insertSample(ADC1BUFA);
		insertSample(ADC1BUFB);
		insertSample(ADC1BUFC);
		insertSample(ADC1BUFD);
		insertSample(ADC1BUFE);
		insertSample(ADC1BUFF);
	}
	else
	{
		// ADC is currently filling buffers 8 - 15.
		insertSample(ADC1BUF0);
		insertSample(ADC1BUF1);
		insertSample(ADC1BUF2);
		insertSample(ADC1BUF3);
		insertSample(ADC1BUF4);
		insertSample(ADC1BUF5);
		insertSample(ADC1BUF6);
		insertSample(ADC1BUF7);
	}
	// The following interrupt flag can only be cleared after reading ADC1BUFx.
	// See the note at the bottom of section 17.7 of the PIC32 family
	// reference manual.
	IFS1bits.AD1IF = 0; // clear interrupt flag
}

/** Begin collecting #SAMPLE_BUFFER_SIZE samples, filling
  * up #adc_sample_buffer. This will return before all the samples have been
  * collected, allowing the caller to do something else while samples are
  * collected in the background. #sample_buffer_full can be used to indicate
  * when #adc_sample_buffer is full.
  *
  * It is okay to call this while the sample buffer is still being filled up.
  * In that case, calling this will reset #sample_buffer_current_index so that
  * the sample buffer will commence filling from the start.
  */
void beginFillingADCBuffer(void)
{
	uint32_t status;

	status = disableInterrupts();
	sample_buffer_current_index = 0;
	sample_buffer_full = 0;
	T3CONbits.ON = 1; // turn timer on
	restoreInterrupts(status);
}

/** Test ADC (and implicitly, the hardware noise source) by displaying some
  * statistics about some ADC samples. */
void testADC(void)
{
	unsigned int i;
	double mean;
	double term;
	double standard_deviation;
	char sbuffer[256];

	// Fill the sample buffer.
	beginFillingADCBuffer();
	while(sample_buffer_full == 0)
	{
		// do nothing
	}

	// Calculate statistics.
	// The conversion factor 3.226 = 3300 mV / 1023.
	mean = 0.0;
	for (i = 0; i < SAMPLE_BUFFER_SIZE; i++)
	{
		mean += (double)adc_sample_buffer[i] * 3.226;
	}
	mean /= SAMPLE_BUFFER_SIZE;
	standard_deviation = 0;
	for (i = 0; i < SAMPLE_BUFFER_SIZE; i++)
	{
		term = ((double)adc_sample_buffer[i] * 3.226) - mean;
		standard_deviation += term * term;
	}
	standard_deviation /= SAMPLE_BUFFER_SIZE;
	standard_deviation = sqrt(standard_deviation);

	// Display statistics.
	writeStringToDisplay("Noise mean:");
	nextLine();
	sprintf(sbuffer, "%g mV", mean);
	writeStringToDisplay(sbuffer);
	nextLine();
	writeStringToDisplay("Noise RMS:");
	nextLine();
	sprintf(sbuffer, "%g mV", standard_deviation);
	writeStringToDisplay(sbuffer);
}
