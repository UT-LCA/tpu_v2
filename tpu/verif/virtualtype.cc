/*****************************************************************************
 * virtualtype
 *
 * This program parses ../vdecode_mips_opcode.txt (note the path dependence
 * means it must be run from the modelsim/ directory) and creates virtualtype.do
 * script for Modelsim to convert numerical instruction opcodes into instruction
 * names.  This makes debugging in the waveforms MUCH easier.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <bits/stdc++.h> 
#include <string>
#include <map>

using namespace std;

int main()
{
  char buf[512];
  char *p;
  char *name;
  map<int,string> instrmap;
  FILE * f;

  f=fopen("../design/vector/vdecode_mips_opcode.txt","r");
  if (!f)
  {
    printf("Error: Failed to open vdecode_mips_opcode.txt\n\n");
    return 1;
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
      printf("Error parsing through quotes\n");

    p++;
    unsigned long match=strtoul(p,NULL,16);

    for(p=name; *p!='\0' ; p++) // Change case to upper case
      if (islower(*p))
        *p-='a'-'A';
      else if (*p=='.')
        *p='_';

    unsigned long x=match;
    x= (x>>18)&(3<<6);    // Put 24&25th bits at location 7:6
    match=match&(0x3f)|x;

    string *s=new string(name);

    instrmap[match]=*s;
  }

  fclose(f);

  //Write results to file: virutaltype.do
  f=fopen("virtualtype.do","w");
  if (!f)
  {
    printf("Error: Failed to open vdecode_mips_opcode.txt\n\n");
    return 1;
  }

  fprintf(f,"virtual type {\n");

  map<int,string>::const_iterator i=instrmap.begin();
  int c=0;
  for (; i!=instrmap.end(); i++)
  {
    //Designate first entry as NOP
    if (i==instrmap.begin() && c==0)
    {
      fprintf(f,"NOP ");
      c++;
    }
    for (; c<i->first; c++)
      fprintf(f,"ERR ");
    fprintf(f,"%s ",i->second.c_str());
    c++;
  }

  fprintf(f,"\n} myVectorType\n");
  fclose(f);

  printf("Done.  Created virtualtype.do\n");

  return 0;
}
