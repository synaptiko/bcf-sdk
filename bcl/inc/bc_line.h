#ifndef _BC_LINE_H
#define _BC_LINE_H

#include <bc_common.h>
#include <bc_fifo.h>

typedef enum
{
    BC_LINE_RETVAL_OK = 0,

    BC_LINE_RETVAL_TOO_LONG = 1,

    BC_LINE_RETVAL_NOT_FOUND = 2

} bc_line_retval_t;

bc_line_retval_t bc_line_get(void *buffer, size_t buffer_size, bc_fifo_t *fifo, char char_end);

#endif /* _BC_LINE_H */
