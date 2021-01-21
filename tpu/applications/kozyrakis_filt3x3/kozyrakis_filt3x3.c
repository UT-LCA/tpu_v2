#include<stdio.h>

#include "data.h"

#define ITERATIONS (1)

volatile int * counter=(void *)0x80000100;
int start_count;
int end_count;

short int K[3][3]={ {300,32765,-300},{32765,32765,32765},{500,32767,-500}};

volatile char out[WIDTH*HEIGHT];

int main()
{
  start_count=*counter;   //Read counter to start benchmarking

  int i;
  for (i=0; i<ITERATIONS; i++)
    filt(HEIGHT,WIDTH,WIDTH,WIDTH,frame1,out,K,8);

  end_count=*counter;   //Read counter to end benchmarking

  printf("-- Target Duration = %d",end_count-start_count);
  fflush(stdout);
  return 0;
}
