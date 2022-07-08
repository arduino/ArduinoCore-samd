#include <samd.h>
#include <stddef.h>
#include <errno.h>

void *_sbrk(int incr)
{
  extern char end; /* End of global variables, defined by the linker */
  static char *brkval = &end ;
  char *prev_brkval = brkval;

  if (brkval + incr > (char *)__get_MSP()) {
    /* Heap and stack collision */
    errno = ENOMEM;
    return (void*) -1;
  }

  brkval += incr ;
  return (void*) prev_brkval;
}
