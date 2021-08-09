#include<stdio.h>

#include "data.h"

volatile char out[WIDTH2*HEIGHT1];

int main()
{
  volatile int * counter=(void *)0x80000100;
  int start_count;
  int end_count;
  int temp;

  //asm volatile("lui %0,0x8000;" : "=r" ( temp ));
  //asm volatile("ori %0,%1,0x0100;" : "=r" ( temp ) : "r" (temp));
  //asm volatile("lw  %0,0(%1);" : "=r" ( start_count) : "r" (temp));
  //asm volatile("lui $28,0x8000;");
  //asm volatile("ori $28,$28,0x0100;");
  //asm volatile("lw  $28,0($28);");
  start_count=*counter;   //Read counter to start benchmarking

  vector_conv(WIDTH1*HEIGHT1,0,200,out);

  //asm volatile("lui %0,0x8000;" : "=r" ( temp ));
  //asm volatile("ori %0,%1,0x0100;" : "=r" ( temp ) : "r" (temp));
  //asm volatile("lw  %0,0(%1);" : "=r" ( end_count) : "r" (temp));
  //asm volatile("lui $28,0x8000;");
  //asm volatile("ori $28,$28,0x0100;");
  //asm volatile("lw  $28,0($28);");
  end_count=*counter;   //Read counter to end benchmarking

  //printf("-- Target Duration = %d\n",end_count-start_count);
  //fflush(stdout);
  return 0;
}
