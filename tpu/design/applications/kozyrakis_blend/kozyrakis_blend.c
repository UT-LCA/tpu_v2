#include<stdio.h>

#include "data.h"

#define WINDOWSIZE (8)

volatile int * counter=(void *)0x80000100;
int start_count;
int end_count;

#define ITERATIONS (1)

volatile char out[WIDTH*HEIGHT];

int main()
{
  //start_count=*counter;   //Read counter to start benchmarking

  int i;
  for (i=0; i<ITERATIONS; i++)
    out[i]=vector_blend(WIDTH*HEIGHT,frame1,frame2,0x80,out);

  //end_count=*counter;   //Read counter to end benchmarking

  //printf("-- Target Duration = %d\n",end_count-start_count);
  //fflush(stdout);
  return 0;
}
