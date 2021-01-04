/***************************************************************************
 * Converts stream of packet bytes into file system requests.
 *
 * Sometimes Modelsim crashes or is incomplete.  You can use this tool to 
 * parse the generated /tmp/modelsim_filesystem.txt file and perform the
 * input/output (Eg. let's you see what was printfed).
 ***************************************************************************/

#include <stdio.h>
#include <stdlib.h>

#include "tm4filesystem.h"

int main(int argc, char **argv)
{
  int c;

  tm4fs_init();

  if (argc!=2)
  {
    fprintf(stderr,"Usage: filesystem_standalone <filename>\n\n");
    return -1;
  }

  FILE * f=fopen(argv[1],"r");
  if (!f)
  {
    fprintf(stderr,"Error: Failed to open file\n\n");
    return -1;
  }

  while (fscanf(f,"%d",&c)==1)
    serviceRequest(c,NULL);

  fclose(f);

  tm4fs_exit();
  return 0;
}

