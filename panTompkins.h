/**
 * ------------------------------------------------------------------------------*
 * File: panTompkins.h                                                           *
 *       Header for an ANSI-C implementation of Pan-Tompkins real-time QRS detec-*
 *       tion algorithm                                                          *
 * Author: Rafael de Moura Moreira <rafaelmmoreira@gmail.com>                    *
 * License: MIT License                                                          *
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
 */

#ifndef PAN_TOMPKINS
#define PAN_TOMPKINS

#include <stdio.h>      // Remove if not using the standard file functions.
#include <stdbool.h>

typedef int dataType;
//typedef enum {false, true} bool;

#define WINDOWSIZE 20   // Integrator window size, in samples. The article recommends 150ms. So, FS*0.15.
// However, you should check empirically if the waveform looks ok.

#define NOSAMPLE -32000 // An indicator that there are no more samples to read. Use an impossible value for a sample.

#define FS 250          // Sampling frequency.

#define DELAY 0		// Delay introduced by the filters. Filter only output samples after this one.
						// Set to 0 if you want to keep the delay. Fixing the delay results in DELAY less samples
						// in the final end result.

#define BUFFSIZE 415    // The size of the buffers (in samples). Must fit more than 1.66 times an RR interval, which
						// typically could be around 1 second.


void panTompkinsInit(const char file_in[] = NULL, const char file_out[] = NULL);
void panTompkinsReset(void);
bool panTompkins(dataType sampleValue);
int panTompkinsGetHR(void);

#ifdef WIN32
bool panTompkinsFlush(void);
#endif

#endif
