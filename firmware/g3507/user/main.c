#include "ti_msp_dl_config.h"
#include "window_can_app.h"

static void appBootLogSend(void)
{
    static const uint8_t bootLog[] = "BOOT_ID=TP3-G3507-STATE-001\r\n";
    uint32_t index;

    for (index = 0U; index < (sizeof(bootLog) - 1U); index++)
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, bootLog[index]);
    }
}

int main(void)
{
    SYSCFG_DL_init();

    if (!window_can_app_init())
    {
        while (1)
        {
            /* Keep initialization failure visible in the debugger. */
        }
    }

    appBootLogSend();

    while (1)
    {
        window_can_app_process();
    }
}
