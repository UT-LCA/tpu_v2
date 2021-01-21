
#include"tm4llist.h"

ListNode *head;

void ListNode_init()
{
  head=NULL;
  ListNode_add(0,stdin);
  ListNode_add(1,stdout);
  ListNode_add(2,stderr);
}

ListNode * ListNode_create(int fileid,FILE*f)
{
  ListNode *x;
  x=(ListNode*)malloc(sizeof(ListNode));
  x->fileid=fileid;
  x->f=f;
  x->next=NULL;
  return x;
}

ListNode * ListNode_search(int fileid)
{
  ListNode *i;
  for (i=head; i!=NULL; i=i->next)
    if (i->fileid==fileid)
      return i;
  return NULL;
}

FILE * ListNode_lookup(int fileid)
{
  ListNode*x=ListNode_search(fileid);
  return (x) ? x->f : NULL;
}

void ListNode_add(int fileid,FILE*f)
{
  ListNode*x=ListNode_create(fileid,f);
  x->next=head;
  head=x;
}

void ListNode_remove(int fileid)
{
  ListNode *i;
  if (!head)
    return;
  if (head->fileid==fileid)
  {
    i=head;
    head=head->next;
    free(i);
    return;
  }
  for (i=head; i && i->next; i=i->next)
    if (i->next->fileid==fileid)
    {
      ListNode * tmp=i->next;
      i->next=i->next->next;
      free(tmp);
      return;
    }
  return;
}

int ListNode_isempty()
{
  return (head!=NULL);
}

FILE * ListNode_pop()
{
  ListNode *i;
  FILE *f=NULL;

  if (!head)
    return NULL;

  i=head;
  f=i->f;
  head=head->next;
  free(i);
  return f;
}



