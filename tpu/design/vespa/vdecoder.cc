/*****************************************************************************
 * vdecoder
 *
 * This program takes vdecode_mips_opcode.txt and generates visa.v which 
 * contains the opcode definitions for each instruction.  It also takes one
 * optional argument which is a file name.  The file should be the instr.dat
 * file generated from a benchmark, this program will parse that file and 
 * disable instructions not found in it.
 *
 * Usage:  vdecoder [path to instr.dat file to subset]
 ****************************************************************************/

#include <stdio.h>
#include <string>
#include <ctype.h>
#include <stdlib.h>
#include <list>
#include <bits/stdc++.h> 

using namespace std;


/*******
 * Takes a stream of opcodes in hexadecimal format and extracts a list of
 * unique vector instructions found within the stream
 *******/
list<unsigned long> get_instrlist(char * fname)
{
  char buf[512];
  char *name;
  FILE * f;
  list<unsigned long> opcode_list;

  f=fopen(fname,"r");
  if (!f)
  {
    fprintf(stderr,"Error: Failed to open file specified\n\n");
    exit(-1);
  }

  while( fgets(buf,512,f))
  {
    unsigned long opcode=strtoul(buf,NULL,16);
    if ((opcode>>26)==18) //If it's a vector instruction
    {
      opcode_list.push_front(opcode);
      //printf("%x\n",opcode);
    }
  }
  opcode_list.unique();

  return opcode_list;
}


bool is_instr_used( list<unsigned long> &opcode_list, 
                    unsigned long match, 
                    unsigned long mask)
{
  list<unsigned long>::const_iterator i=opcode_list.begin();
  for (; i!=opcode_list.end(); i++)
    if ( (mask & *i) == match)
      return true;
  return false;
}


/****************
 * This function takes a pointer to a list of opcodes it should make
 * instruction definitions for.
 *
 * If opcode_list is NULL it creates instruction definitions for all
 * instructions.
 *
 * Else, it will generate instruction definitions for only those in the list
 * and will othewise generate an 'bxxxxxxxxxx definition for instructions not
 * found in the list.
 *****************/

void generate_visa( list<unsigned long> *opcode_list)
{
  char buf[512];
  char *p;
  char *name;
  int i;
  FILE * f;

  f=fopen("vdecode_mips_opcode.txt","r");
  if (!f)
  {
    fprintf(stderr,"Error: Failed to open vdecode_mips_opcode.txt\n\n");
    exit(-1);
  }

  while( fgets(buf,512,f))
  {
    if (buf[0]=='\0' || buf[0]=='\n')
      break;

    p=buf;

    p=strchr(p,'"');
    p++;
    name=p;
    p=strchr(p,'"');
    *p='\0';
    p++;
    p=strchr(p,'"');
    p++;
    p=strchr(p,'"');
    p++;

    if (p[0]!=',')
      fprintf(stderr,"Error parsing through quotes\n");

    p++;
    unsigned long match=strtoul(p,NULL,16);

    p=strchr(p,',');

    p++;
    unsigned long mask=strtoul(p,NULL,16);

    for(p=name; *p!='\0' ; p++)
      if (islower(*p))
        *p-='a'-'A';
      else if (*p=='.')
        *p='_';

    char literal[25];
    p=literal;

    for (i=25; i>=22; i--)
      if (mask & (1<<i))
        if (match & (1<<i))
          *p++='1';
        else
          *p++='0';
      else
        *p++='z';

    for (i=5; i>=0; i--)
      if (mask & (1<<i))
        if (match & (1<<i))
          *p++='1';
        else
          *p++='0';
      else
        *p++='z';

    *p='\0';

    //printf("name=%s match=%x mask=%x literal=%s\n",name,match,mask,literal);

    printf("parameter COP2_%s",name);
    for (i=15-strlen(name); i>0; i--)
      printf(" ");

    if (!opcode_list || is_instr_used(*opcode_list,match,mask))
      printf("= 'b%s;\n",literal);
    else
      printf("= 'bxxxxxxxxxx;\n");

  }

  fclose(f);

}


int main(int argc, char * argv[])
{

  if (argc<2)
    generate_visa(NULL);
  else
  {
    list<unsigned long> opcode_list=get_instrlist(argv[1]);    
    generate_visa(&opcode_list);
  }

  return 0;
}
