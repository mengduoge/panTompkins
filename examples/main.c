#include <stdio.h>
#include "panTompkins.h"

#define NOSAMPLE -32000

extern dataType input(void);
extern void output(int out);

int main()
{
	dataType sample;
	int i;

	init("..\\examples\\test_input.csv", "..\\examples\\test_output.csv");

	while ((sample = input()) != NOSAMPLE)
		output(panTompkins(sample));

	for (i = 1; i < BUFFSIZE; i++)
		output(panTompkinsFlush());

	return 0;
}
