/**
 * ------------------------------------------------------------------------------*
 * File: panTompkins.c                                                           *
 *       ANSI-C implementation of Pan-Tompkins real-time QRS detection algorithm *
 * Author: Rafael de Moura Moreira <rafaelmmoreira@gmail.com>                    *
 * License: MIT License                                                          *
 * ------------------------------------------------------------------------------*
 * ---------------------------------- HISTORY ---------------------------------- *
 *    date   |    author    |                     description                    *
 * ----------| -------------| ---------------------------------------------------*
 * 2019/04/11| Rafael M. M. | - Fixed moving-window integral.                    *
 *           |              | - Fixed how to find the correct sample with the    *
 *           |              | last QRS.                                          *
 *           |              | - Replaced constant value in code by its #define.  *
 *           |              | - Added some casting on comparisons to get rid of  *
 *           |              | compiler warnings.                                 *
 * 2019/04/15| Rafael M. M. | - Removed delay added to the output by the filters.*
 *           |              | - Fixed multiple detection of the same peak.       *
 * 2019/04/16| Rafael M. M. | - Added output buffer to correctly output a peak   *
 *           |              | found by back searching using the 2nd thresholds.  *
 * 2019/04/23| Rafael M. M. | - Improved comparison of slopes.                   *
 *           |              | - Fixed formula to obtain the correct sample from  *
 *           |              | the buffer on the back search.                     *
 * ------------------------------------------------------------------------------*
 * MIT License                                                                   *
 *                                                                               *
 * Copyright (c) 2018 Rafael de Moura Moreira                                    *
 *                                                                               *
 * Permission is hereby granted, free of charge, to any person obtaining a copy  *
 * of this software and associated documentation files (the "Software"), to deal *
 * in the Software without restriction, including without limitation the rights  *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
 * copies of the Software, and to permit persons to whom the Software is         *
 * furnished to do so, subject to the following conditions:                      *
 *                                                                               *
 * The above copyright notice and this permission notice shall be included in all*
 * copies or substantial portions of the Software.                               *
 *                                                                               *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE *
 * SOFTWARE.                                                                     *
 *-------------------------------------------------------------------------------*
 * Description                                                                   *
 *                                                                               *
 * The main goal of this implementation is to be easy to port to different opera-*
 * ting systems, as well as different processors and microcontrollers, including *
 * embedded systems. It can work both online or offline, depending on whether all*
 * the samples are available or not - it can be adjusted on the input function.  *
 *                                                                               *
 * The main function, panTompkings(), calls input() to get the next sample and   *
 * store it in a buffer. Then it runs through a chain of filters: DC block, low  *
 * pass @ 15 Hz and high pass @ 5Hz. The filtered signal goes both through a de- *
 * rivative filter, which output is then squared, and through a windowed-integra-*
 * tor.                                                                          *
 *                                                                               *
 * For a signal peak to be recognized as a fiducial point, its correspondent va- *
 * lue on both the filtered signal and the integrator must be above a certain    *
 * threshold. Additionally, there are time-restraints to prevent a T-wave from   *
 * being mistakenly identified as an R-peak: a hard 200ms restrain (a new peak   *
 * 200ms from the previous one is, necessarily, a T-wave) and a soft 360ms res-  *
 * train (the peak's squared slope must also be very high to be considered as a  *
 * real peak).                                                                   *
 *                                                                               *
 * When a peak candidate is discarded, its value is used to update the noise     *
 * thresholds - which are also used to estimate the peak thresholds.             *
 *                                                                               *
 * Two buffers keep 8 RR-intervals to calculate RR-averages: one of them keeps   *
 * the last 8 RR-intervals, while the other keeps only the RR-intervals that res-*
 * pect certain restrictions. If both averages are equal, the heart pace is con- *
 * sidered normal. If the heart rate isn't normal, the thresholds change to make *
 * it easier to detect possible weaker peaks. If no peak is detected for a long  *
 * period of time, the thresholds also change and the last discarded peak candi- *
 * date is reconsidered.                                                         *
 *-------------------------------------------------------------------------------*
 * Instructions                                                                  *
 *                                                                               *
 * Here's what you should change to adjust the code to your needs:               *
 *                                                                               *
 * On panTompkins.h:                                                             *
 * - typedef int dataType;                                                       *
 * Change it from 'int' to whatever format your data is (float, unsigned int etc)*
 *                                                                               *
 * On both panTompkins.h and panTompkins.c:                                      *
 * - void init(char file_in[], char file_out[]);                                 *
 * This function is meant to do any kind of initial setup, such as starting a    *
 * serial connection with an ECG sensor. Change its parameters to whatever info  *
 * you need and its content. The test version included here loads 2 plain text   *
 * files: an input file, with the signal as a list of integer numbers in ASCII   *
 * format and an output file where either 0 or 1 will be written for each sample,*
 * whether a peak was detected or not.                                           *
 *                                                                               *
 * On panTompkins.c:                                                             *
 * - #define WINDOWSIZE                                                          *
 * Defines the size of the integration window. The original authors suggest on   *
 * their 1985 paper a 150ms window.                                              *
 *                                                                               *
 * - #define FS                                                                  *
 * Defines the sampling frequency.                                               *
 *                                                                               *
 * - #define NOSAMPLE                                                            *
 * A value to indicate you don't have any more samples to read. Choose a value   *
 * which a sample couldn't possibly have (e.g.: a negative value if your A/D con-*
 * verter only works with positive integers).                                    *
 *                                                                               *
 * - #define BUFFSIZE                                                            *
 * The size of the signal buffers. It should fit at least 1.66 times an RR-inter-*
 * val. Heart beats should be between 60 and 80 BPS for humans. So, considering  *
 * 1.66 times 1 second should be safe.                                           *
 *                                                                               *
 * - #define DELAY 22                                                            *
 * The delay introduced to the output signal. The first DELAY samples will be ig-*
 * nored, as the filters add a delay to the output signal, causing a mismatch    *
 * between the input and output signals. It's easier to compare them this way.   *
 * If you need them both to have the same amount of samples, set this to 0. If   *
 * you're working with different filters and/or sampling rates, you might need to*
 * adjust this value.                                                            *
 *                                                                               *
 * - #include <stdio.h>                                                          *
 * The file, as it is, both gets its inputs and sends its outputs to files. It   *
 * works on both Windows and Linux. If your source isn't a file, and/or your sys-*
 * tem doesn't have the <stdio.h> header, remove it.                             *
 * Include any other headers you might need to make your implementation work,    *
 * such as hardware libraries provided by your microcontroller manufacturer.     *
 *                                                                               *
 * - The input() function                                                        *
 * Change it to get the next sample from your source (a file, a serial device etc*
 * previously set up in your init() function. Return the sample value or NOSAMPLE*
 * if there are no more available samples.                                       *
 *                                                                               *
 * - The output() function                                                       *
 * Change it to output whatever you see fit, however you see fit: an RR-interval *
 * (which can be sent as a parameter to your function using the RR arrays), the  *
 * index of sample or timestamp which caused a R peak, whether a sample was a R  *
 * peak or not etc, and it can be written on a file, displayed on screen, blink a*
 * LED etc.                                                                      *
 *                                                                               *
 * - The panTompkins() function                                                  *
 * This function is almost entirely ANSI C, which means it should work as is on  *
 * most platforms. The only lines you really have to change are the fclose() ones*
 * at the very end, which are only here to allow testing of the code on systems  *
 * such as Windows and Linux for PC. You may wish to create extra variables or   *
 * increase the buffers' size as you see fit to extract different kinds of infor-*
 * mation to output, or fine tune the detection as you see fit - such as adding  *
 * different conditions for verification, initializing self-updating variables   *
 * with empirically-obtained values or changing the filters.                     *
 *-------------------------------------------------------------------------------*
 */

#define WINDOWSIZE 20   // Integrator window size, in samples. The article recommends 150ms. So, FS*0.15.
						// However, you should check empirically if the waveform looks ok.
#define NOSAMPLE -32000 // An indicator that there are no more samples to read. Use an impossible value for a sample.
#define FS 360          // Sampling frequency.
                        // typically could be around 1 second.

#define DELAY 22		// Delay introduced by the filters. Filter only output samples after this one.
						// Set to 0 if you want to keep the delay. Fixing the delay results in DELAY less samples
						// in the final end result.

#include "panTompkins.h"
#include <stdio.h>      // Remove if not using the standard file functions.


FILE *fin, *fout;       // Remove them if not using files and <stdio.h>.

static struct {
	dataType signal[BUFFSIZE], dcblock[BUFFSIZE], lowpass[BUFFSIZE], highpass[BUFFSIZE];
	dataType derivative[BUFFSIZE], squared[BUFFSIZE], integral[BUFFSIZE], outputSignal[BUFFSIZE];
	int rr1[8], rr2[8];
	int rravg1, rravg2, rrlow, rrhigh, rrmiss;
	long unsigned int sample, lastQRS, lastSlope, currentSlope;
	int current;
	dataType peak_i, peak_f, threshold_i1, threshold_i2, threshold_f1, threshold_f2;
	dataType spk_i, spk_f, npk_i, npk_f;
	bool regular, prevRegular;
	int flushIndex;
} pt;

static bool panTompkinsGetOutput(bool qrs)
{
	pt.outputSignal[pt.current] = qrs;
	if (pt.sample > DELAY + BUFFSIZE)
		return pt.outputSignal[0];
	return false;
}

void panTompkinsReset(void)
{
	long unsigned int i;

	for (i = 0; i < 8; i++)
	{
		pt.rr1[i] = 0;
		pt.rr2[i] = 0;
	}
	pt.rravg1 = 0;
	pt.rravg2 = 0;
	pt.rrlow = 0;
	pt.rrhigh = 0;
	pt.rrmiss = 0;
	pt.sample = 0;
	pt.lastQRS = 0;
	pt.lastSlope = 0;
	pt.currentSlope = 0;
	pt.current = 0;
	pt.peak_i = 0;
	pt.peak_f = 0;
	pt.threshold_i1 = 0;
	pt.threshold_i2 = 0;
	pt.threshold_f1 = 0;
	pt.threshold_f2 = 0;
	pt.spk_i = 0;
	pt.spk_f = 0;
	pt.npk_i = 0;
	pt.npk_f = 0;
	pt.regular = true;
	pt.flushIndex = 0;
}

bool panTompkinsFlush(void)
{
	pt.flushIndex++;
	if (pt.flushIndex < BUFFSIZE)
		return pt.outputSignal[pt.flushIndex];
	return false;
}

/*
    Use this function for any kind of setup you need before getting samples.
    This is a good place to open a file, initialize your hardware and/or open
    a serial connection.
    Remember to update its parameters on the panTompkins.h file as well.
*/
void init(const char file_in[], const char file_out[])
{
	fin = fopen(file_in, "r");
	fout = fopen(file_out, "w+");
	panTompkinsReset();
}

/*
    Use this function to read and return the next sample (from file, serial,
    A/D converter etc) and put it in a suitable, numeric format. Return the
    sample, or NOSAMPLE if there are no more samples.
*/
dataType input()
{
	int num = NOSAMPLE;
	if (!feof(fin))
		fscanf(fin, "%d", &num);

	return num;
}

/*
    Use this function to output the information you see fit (last RR-interval,
    sample index which triggered a peak detection, whether each sample was a R
    peak (1) or not (0) etc), in whatever way you see fit (write on screen, write
    on file, blink a LED, call other functions to do other kinds of processing,
    such as feature extraction etc). Change its parameters to receive the necessary
    information to output.
*/
void output(int out)
{
	fprintf(fout, "%d\n", out);
}

/*
    Processes one AD sample and returns whether an R peak was detected for the
    delayed output sample (same timing as the original output() calls).
    Call panTompkinsReset() before the first sample. Call panTompkinsFlush()
    after the last sample to drain the remaining buffered outputs.
*/
bool panTompkins(dataType sampleValue)
{
	long unsigned int i, j;
	bool qrs;
	bool prevRegular;

	// Test if the buffers are full.
	// If they are, shift them, discarding the oldest sample and adding the new one at the end.
	// Else, just put the newest sample in the next free position.
	// Update 'current' so that the program knows where's the newest sample.
	if (pt.sample >= BUFFSIZE)
	{
		for (i = 0; i < BUFFSIZE - 1; i++)
		{
			pt.signal[i] = pt.signal[i+1];
			pt.dcblock[i] = pt.dcblock[i+1];
			pt.lowpass[i] = pt.lowpass[i+1];
			pt.highpass[i] = pt.highpass[i+1];
			pt.derivative[i] = pt.derivative[i+1];
			pt.squared[i] = pt.squared[i+1];
			pt.integral[i] = pt.integral[i+1];
			pt.outputSignal[i] = pt.outputSignal[i+1];
		}
		pt.current = BUFFSIZE - 1;
	}
	else
	{
		pt.current = (int)pt.sample;
	}
	pt.signal[pt.current] = sampleValue;
	pt.sample++;

	// DC Block filter
	if (pt.current >= 1)
		pt.dcblock[pt.current] = pt.signal[pt.current] - pt.signal[pt.current-1] + 0.995*pt.dcblock[pt.current-1];
	else
		pt.dcblock[pt.current] = 0;

	// Low Pass filter
	pt.lowpass[pt.current] = pt.dcblock[pt.current];
	if (pt.current >= 1)
		pt.lowpass[pt.current] += 2*pt.lowpass[pt.current-1];
	if (pt.current >= 2)
		pt.lowpass[pt.current] -= pt.lowpass[pt.current-2];
	if (pt.current >= 6)
		pt.lowpass[pt.current] -= 2*pt.dcblock[pt.current-6];
	if (pt.current >= 12)
		pt.lowpass[pt.current] += pt.dcblock[pt.current-12];

	// High Pass filter
	pt.highpass[pt.current] = -pt.lowpass[pt.current];
	if (pt.current >= 1)
		pt.highpass[pt.current] -= pt.highpass[pt.current-1];
	if (pt.current >= 16)
		pt.highpass[pt.current] += 32*pt.lowpass[pt.current-16];
	if (pt.current >= 32)
		pt.highpass[pt.current] += pt.lowpass[pt.current-32];

	// Derivative filter
	pt.derivative[pt.current] = pt.highpass[pt.current];
	if (pt.current > 0)
		pt.derivative[pt.current] -= pt.highpass[pt.current-1];

	pt.squared[pt.current] = pt.derivative[pt.current]*pt.derivative[pt.current];

	// Moving-Window Integration
	pt.integral[pt.current] = 0;
	for (i = 0; i < WINDOWSIZE; i++)
	{
		if (pt.current >= (int)i)
			pt.integral[pt.current] += pt.squared[pt.current - i];
		else
			break;
	}
	pt.integral[pt.current] /= (dataType)i;

	qrs = false;

	if (pt.integral[pt.current] >= pt.threshold_i1 || pt.highpass[pt.current] >= pt.threshold_f1)
	{
		pt.peak_i = pt.integral[pt.current];
		pt.peak_f = pt.highpass[pt.current];
	}

	if ((pt.integral[pt.current] >= pt.threshold_i1) && (pt.highpass[pt.current] >= pt.threshold_f1))
	{
		if (pt.sample > pt.lastQRS + FS/5)
		{
			if (pt.sample <= pt.lastQRS + (long unsigned int)(0.36*FS))
			{
				pt.currentSlope = 0;
				for (j = pt.current - 10; j <= (long unsigned int)pt.current; j++)
					if (pt.squared[j] > pt.currentSlope)
						pt.currentSlope = pt.squared[j];

				if (pt.currentSlope <= (dataType)(pt.lastSlope/2))
				{
					qrs = false;
				}
				else
				{
					pt.spk_i = 0.125*pt.peak_i + 0.875*pt.spk_i;
					pt.threshold_i1 = pt.npk_i + 0.25*(pt.spk_i - pt.npk_i);
					pt.threshold_i2 = 0.5*pt.threshold_i1;

					pt.spk_f = 0.125*pt.peak_f + 0.875*pt.spk_f;
					pt.threshold_f1 = pt.npk_f + 0.25*(pt.spk_f - pt.npk_f);
					pt.threshold_f2 = 0.5*pt.threshold_f1;

					pt.lastSlope = pt.currentSlope;
					qrs = true;
				}
			}
			else
			{
				pt.currentSlope = 0;
				for (j = pt.current - 10; j <= (long unsigned int)pt.current; j++)
					if (pt.squared[j] > pt.currentSlope)
						pt.currentSlope = pt.squared[j];

				pt.spk_i = 0.125*pt.peak_i + 0.875*pt.spk_i;
				pt.threshold_i1 = pt.npk_i + 0.25*(pt.spk_i - pt.npk_i);
				pt.threshold_i2 = 0.5*pt.threshold_i1;

				pt.spk_f = 0.125*pt.peak_f + 0.875*pt.spk_f;
				pt.threshold_f1 = pt.npk_f + 0.25*(pt.spk_f - pt.npk_f);
				pt.threshold_f2 = 0.5*pt.threshold_f1;

				pt.lastSlope = pt.currentSlope;
				qrs = true;
			}
		}
		else
		{
			pt.peak_i = pt.integral[pt.current];
			pt.npk_i = 0.125*pt.peak_i + 0.875*pt.npk_i;
			pt.threshold_i1 = pt.npk_i + 0.25*(pt.spk_i - pt.npk_i);
			pt.threshold_i2 = 0.5*pt.threshold_i1;
			pt.peak_f = pt.highpass[pt.current];
			pt.npk_f = 0.125*pt.peak_f + 0.875*pt.npk_f;
			pt.threshold_f1 = pt.npk_f + 0.25*(pt.spk_f - pt.npk_f);
			pt.threshold_f2 = 0.5*pt.threshold_f1;
			return panTompkinsGetOutput(false);
		}
	}

	if (qrs)
	{
		pt.rravg1 = 0;
		for (i = 0; i < 7; i++)
		{
			pt.rr1[i] = pt.rr1[i+1];
			pt.rravg1 += pt.rr1[i];
		}
		pt.rr1[7] = (int)(pt.sample - pt.lastQRS);
		pt.lastQRS = pt.sample;
		pt.rravg1 += pt.rr1[7];
		pt.rravg1 *= 0.125;

		if ((pt.rr1[7] >= pt.rrlow) && (pt.rr1[7] <= pt.rrhigh))
		{
			pt.rravg2 = 0;
			for (i = 0; i < 7; i++)
			{
				pt.rr2[i] = pt.rr2[i+1];
				pt.rravg2 += pt.rr2[i];
			}
			pt.rr2[7] = pt.rr1[7];
			pt.rravg2 += pt.rr2[7];
			pt.rravg2 *= 0.125;
			pt.rrlow = (int)(0.92*pt.rravg2);
			pt.rrhigh = (int)(1.16*pt.rravg2);
			pt.rrmiss = (int)(1.66*pt.rravg2);
		}

		prevRegular = pt.regular;
		if (pt.rravg1 == pt.rravg2)
		{
			pt.regular = true;
		}
		else
		{
			pt.regular = false;
			if (prevRegular)
			{
				pt.threshold_i1 /= 2;
				pt.threshold_f1 /= 2;
			}
		}
	}
	else
	{
		if ((pt.sample - pt.lastQRS > (long unsigned int)pt.rrmiss) && (pt.sample > pt.lastQRS + FS/5))
		{
			for (i = pt.current - (pt.sample - pt.lastQRS) + FS/5; i < (long unsigned int)pt.current; i++)
			{
				if ((pt.integral[i] > pt.threshold_i2) && (pt.highpass[i] > pt.threshold_f2))
				{
					pt.currentSlope = 0;
					for (j = i - 10; j <= i; j++)
						if (pt.squared[j] > pt.currentSlope)
							pt.currentSlope = pt.squared[j];

					if ((pt.currentSlope < (dataType)(pt.lastSlope/2)) && (i + pt.sample) < pt.lastQRS + 0.36*pt.lastQRS)
					{
						qrs = false;
					}
					else
   					{
						pt.peak_i = pt.integral[i];
						pt.peak_f = pt.highpass[i];
						pt.spk_i = 0.25*pt.peak_i+ 0.75*pt.spk_i;
						pt.spk_f = 0.25*pt.peak_f + 0.75*pt.spk_f;
						pt.threshold_i1 = pt.npk_i + 0.25*(pt.spk_i - pt.npk_i);
						pt.threshold_i2 = 0.5*pt.threshold_i1;
						pt.lastSlope = pt.currentSlope;
						pt.threshold_f1 = pt.npk_f + 0.25*(pt.spk_f - pt.npk_f);
						pt.threshold_f2 = 0.5*pt.threshold_f1;

						pt.rravg1 = 0;
						for (j = 0; j < 7; j++)
						{
							pt.rr1[j] = pt.rr1[j+1];
							pt.rravg1 += pt.rr1[j];
						}
						pt.rr1[7] = (int)(pt.sample - (pt.current - (int)i) - pt.lastQRS);
						qrs = true;
						pt.lastQRS = pt.sample - (pt.current - i);
						pt.rravg1 += pt.rr1[7];
						pt.rravg1 *= 0.125;

						if ((pt.rr1[7] >= pt.rrlow) && (pt.rr1[7] <= pt.rrhigh))
						{
							pt.rravg2 = 0;
							for (j = 0; j < 7; j++)
							{
								pt.rr2[j] = pt.rr2[j+1];
								pt.rravg2 += pt.rr2[j];
							}
							pt.rr2[7] = pt.rr1[7];
							pt.rravg2 += pt.rr2[7];
							pt.rravg2 *= 0.125;
							pt.rrlow = (int)(0.92*pt.rravg2);
							pt.rrhigh = (int)(1.16*pt.rravg2);
							pt.rrmiss = (int)(1.66*pt.rravg2);
						}

						prevRegular = pt.regular;
						if (pt.rravg1 == pt.rravg2)
						{
							pt.regular = true;
						}
						else
						{
							pt.regular = false;
							if (prevRegular)
							{
								pt.threshold_i1 /= 2;
								pt.threshold_f1 /= 2;
							}
						}

						break;
					}
				}
			}

			if (qrs)
			{
				pt.outputSignal[pt.current] = false;
				pt.outputSignal[i] = true;
				if (pt.sample > DELAY + BUFFSIZE)
					return pt.outputSignal[0];
				return false;
			}
		}

		if (!qrs)
		{
			if ((pt.integral[pt.current] >= pt.threshold_i1) || (pt.highpass[pt.current] >= pt.threshold_f1))
			{
				pt.peak_i = pt.integral[pt.current];
				pt.npk_i = 0.125*pt.peak_i + 0.875*pt.npk_i;
				pt.threshold_i1 = pt.npk_i + 0.25*(pt.spk_i - pt.npk_i);
				pt.threshold_i2 = 0.5*pt.threshold_i1;
				pt.peak_f = pt.highpass[pt.current];
				pt.npk_f = 0.125*pt.peak_f + 0.875*pt.npk_f;
				pt.threshold_f1 = pt.npk_f + 0.25*(pt.spk_f - pt.npk_f);
				pt.threshold_f2 = 0.5*pt.threshold_f1;
			}
		}
	}

	return panTompkinsGetOutput(qrs);
}
