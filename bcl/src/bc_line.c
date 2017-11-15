#include <bc_line.h>

bc_line_retval_t bc_line_get(void *buffer, size_t buffer_size, bc_fifo_t *fifo, char char_end)
{
    char *p = buffer;
    size_t length = 0;

    while (true)
    {
        char rx_character;

        if (bc_fifo_read(fifo, &rx_character, 1) == 0)
        {
            return BC_LINE_RETVAL_NOT_FOUND;
        }

        *p++ = rx_character;
        length++;

        if (rx_character == char_end)
        {
            *p = '\0';

            break;
        }

        if (length == buffer_size - 1)
        {
            return BC_LINE_RETVAL_TOO_LONG;
        }
    }

    return BC_LINE_RETVAL_OK;
}
