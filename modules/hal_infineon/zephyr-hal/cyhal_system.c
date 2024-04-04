

#include <zephyr/kernel.h>
#include <cy_result.h>

cy_rslt_t cyhal_system_delay_ms(uint32_t milliseconds)
{
    k_sleep(K_MSEC(milliseconds));

    return 0;
}
