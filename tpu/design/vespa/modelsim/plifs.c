#include "veriuser.h"
#include "vpi_user.h"
//#include "vcs_acc_user.h"
#include "tm4filesystem.h"

static int is_initialized=0;

static PLI_INT32 plifs()
{
  char c;

  if (!is_initialized) 
  {
    tm4fs_init();
    is_initialized=1;
  }

  int x=tf_getp(1);

  serviceRequest((char)x,&c);

  tf_putp(2,(int)c);

  return 0;
}

static PLI_INT32 tm4fsexit()
{
  if (is_initialized) 
    tm4fs_exit();
  return 0;
}

//s_tfcell veriusertfs[] = {
//  {usertask, 0, 0, 0, plifs, 0, "$plifs"},
//  {usertask, 0, 0, 0, tm4fsexit, 0, "$tm4fsexit"},
//  {0} /* last entry must be 0 */
//};
