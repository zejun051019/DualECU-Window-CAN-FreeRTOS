/* Proves native C compilation, linking and execution, not target behavior. */
#include <stdint.h>
#include <stdio.h>

int main(void)
{
    volatile uint32_t before = UINT32_MAX - 4U;
    volatile uint32_t after = 3U;
    if ((uint32_t)(after - before) != 8U) {
        return 1;
    }
    puts("host C smoke passed");
    return 0;
}
