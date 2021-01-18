#include<stdio.h>

#include "data.h"


volatile int * counter=(void *)0x80000100;
int start_count;
int end_count;


volatile char out[WIDTH*HEIGHT];

int main()
{
  //start_count=*counter;   //Read counter to start benchmarking

  vector_add(WIDTH*HEIGHT,frame1,frame2,out);

  //end_count=*counter;   //Read counter to end benchmarking

  return 0;
}
