#ifndef _TM4LLIST_H_
#define _TM4LLIST_H_

#include<stdio.h>
#include<stdlib.h>

struct ListNode_t {
  int fileid;
  FILE * f;
  struct ListNode_t* next;
};
typedef struct ListNode_t ListNode;

void ListNode_init();
FILE * ListNode_lookup(int fileid);
ListNode * ListNode_search(int fileid);
ListNode * ListNode_create(int fileid,FILE*f);
void ListNode_add(int fileid,FILE*f);
void ListNode_remove(int fileid);
FILE * ListNode_pop();
int ListNode_isempty();

#endif
