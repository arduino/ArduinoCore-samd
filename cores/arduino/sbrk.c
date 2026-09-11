#include <samd.h>
#include <stddef.h>
#include <errno.h>

/* How much free space to keep between the stack and the heap by malloc.
 * Can be changed by the application at any time. */
size_t __malloc_margin = 64;

void *_sbrk(int incr)
{
  extern char end; /* End of global variables, defined by the linker */
  static char *brkval = &end ;
  char *prev_brkval = brkval;

  if (brkval + incr + __malloc_margin > (char *)__get_MSP()) {
    /* Heap and stack collision */
    errno = ENOMEM;
    return (void*) -1;
  }

  brkval += incr ;
  return (void*) prev_brkval;
}
