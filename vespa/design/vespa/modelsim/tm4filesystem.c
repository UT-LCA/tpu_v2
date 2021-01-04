#include "tm4filesystem.h"
#include "tm4llist.h"

#include<stdio.h>
#include<string.h>

//#define DEBUG

#define BUFSIZE 256

typedef enum{S_IDLE,S_START1,S_START2,S_LENGTH,S_TRANSFER,S_END,S_READLENGTH,S_READ} t_state;
t_state state=S_IDLE;

Intchar_t intchar;

int length=0;
int substate=0;
char *data;
int myfileno;
char cmd;
FILE *f_read;          //file for reading

int port_fs_writedata=0;
int port_fs_readdata=0;
int nextfileid=3;

void tm4fs_init()
{
  ListNode_init();
}

/**
 * Pop all files off the list and flush and close them 
 * This includes stdout,stdin, stderr
 */
void tm4fs_exit()
{
  while(!ListNode_isempty())
  {
    FILE *f=ListNode_pop();
    fflush(f);
    fclose(f);
  }
}

// returns true if outchar is valid (used to output characters to be read)
void serviceRequest(char c, char *outchar)
{
#ifdef DEBUG
printf("State: %d",state); fflush(stdout);
#endif
  switch(state)
  {
    case S_IDLE:
      if ((unsigned char)c==START1)
        state=S_START1;
      break;
    case S_START1:
      if ((unsigned char)c==START2)
        state=S_START2;
      break;
    case S_START2:
      substate=0;
      intchar.buf[substate]=c;
      state=S_LENGTH;
      break;
    case S_LENGTH:
      intchar.buf[++substate]=c;
      if (substate>=3)
      {
        state=S_TRANSFER;
        length=intchar.num;
        data=(char*)malloc((length)*sizeof(char));
        substate=0;
      }
      break;
    case S_TRANSFER:
      data[substate++]=c;
      if (substate>=length)
      {
        state=S_END;
      }
      break;
    case S_END:
      if ((unsigned char)c==END)
      {
        // 6. Extract command 
        cmd=data[0];

        // 7. Extract file id
        memcpy(intchar.buf,&data[1],4);  
        myfileno=intchar.num;

#ifdef DEBUG
        if (cmd==CMD_OPEN)
          printf("cmd = %d, flags=%d\n",cmd,myfileno);
        else
          printf("cmd = %d, fileid=%d\n",cmd,myfileno);
#endif

        // 8. Service command
        state=S_IDLE;     // Default to idle, only Read doesn't go to IDLE
        switch(cmd)
        {
          case CMD_NOP:
            free(data);
            break;
          case CMD_OPEN:
            serviceOpen(&data[1+4],myfileno);   //note myfileno actually has flags
            free(data);
            break;
          case CMD_WRITE:
            serviceWrite(myfileno,&data[1+4],length-1-4); //exclude cmd and myfileno
            free(data);
            break;
          case CMD_READ:
#ifdef DEBUG
            printf("Reading from file number %d",myfileno);
#endif
            memcpy(intchar.buf,&data[1+4],4);  
#ifdef DEBUG
            printf(" total of %d bytes\n",intchar.num);
            fflush(stdout);
#endif
            free(data);
            length=serviceRead(myfileno,intchar.num);   // get actual bytes read
            intchar.num=length;
            substate=0;
            state=S_READLENGTH;
            break;
          case CMD_CLOSE:
            serviceClose(myfileno);
            free(data);
            break;
          default:
            fprintf(stderr,"Uknown Command received: %d\n",cmd);
        }
      }
      else
      {
        fprintf(stderr,"Error: Packet incomplete - END not found\n"); 
        fflush(stderr);
      }
      break;
    case S_READLENGTH:
      *outchar=intchar.buf[substate++];
      if (substate >= 4)
      {
        substate=0;
        state=S_READ;
      }
      break;
    case S_READ:
      *outchar=data[substate++];
      if (substate >= intchar.num)
      {
        free(data);
        state=S_IDLE;
      }
      break;
    default: state=S_IDLE;
      break;
  }

#ifdef DEBUG
printf(" char servicing: %x, char out: %x\n",c,(outchar) ? *outchar : 0); fflush(stdout);
printf("\n");
#endif
  return;
}

//"a" gets 0010 0000 1001
void serviceOpen(char *filename, int flags)
{
  char buf[8];
  int i=0;

  if (!flags)
    buf[i++]='r';  //read by default (O_RDONLY = 0!!!)
  else if ((flags&O_WRONLY) && (flags&O_TRUNC) && !(flags&O_APPEND) && (flags&O_CREAT))
    buf[i++]='w';
  else if ((flags&O_WRONLY) && !(flags&O_TRUNC) && (flags&O_APPEND) && (flags&O_CREAT))
    buf[i++]='a';
  else
  {
    fprintf(stderr,"Unknown mode specified for file open\n");
    fflush(stderr);
  }
  buf[i++]='\0';

  FILE *f=fopen(filename,buf);

#ifdef DEBUG
printf("File %s with mode <%s>, handle %x, fileid %d\n",filename,buf,f,nextfileid);
fflush(stdout);
#endif

  if (!f)
  {
    fprintf(stderr,"Failed to open file %s with mode %s\n",filename,buf);
    fflush(stderr);
  }
  else
    ListNode_add(nextfileid++,f);
}

void serviceWrite(int myfileno, char *data, int length)
{
  FILE *f=ListNode_lookup(myfileno);
  fwrite(data,sizeof(char),length,f);
  fflush(f);
}

int serviceRead(int myfileno, int length)
{
  f_read=ListNode_lookup(myfileno);
  data=(char*)malloc(length*sizeof(char));

  int bytesread=fread(data,sizeof(char),length,f_read);

#ifdef DEBUG
printf("Performed fread on file number %d of %d bytes\n",myfileno,length);
printf("<data written to TM4>: %s",data);
printf("</data written to TM4>\n");
fflush(stdout);
#endif

  return bytesread;
}

void serviceClose(int myfileno)
{
#ifdef DEBUG
printf("Closing file number %d\n",myfileno);
fflush(stdout);
#endif
  FILE *f=ListNode_lookup(myfileno);
  fclose(f);
  ListNode_remove(myfileno);
}

