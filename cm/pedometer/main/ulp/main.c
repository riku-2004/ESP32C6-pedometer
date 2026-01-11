#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include "soc/soc_caps.h"
#include "hal/lp_core_ll.h"
#include "riscv/rv_utils.h"
#include "riscv/rvruntime-frames.h"
#include "ulp_lp_core.h"
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_interrupts.h"
#include "ulp_lp_core_lp_timer_shared.h"

int main (void)
{
    lp_core_i2c_master_write_to_device(0, 0, NULL, 0, 0);
    lp_core_i2c_master_read_from_device(0, 0, NULL, 0, 0);
    return 0;
}

