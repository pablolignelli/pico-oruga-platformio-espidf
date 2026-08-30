#include "soc/gpio_num.h"
#ifndef __INFRASTRUCTURE_H
#define __INFRASTRUCTURE_H

class Infrastructure {
public:
    static bool setup(gpio_num_t motor_emergency_stop, gpio_num_t emergency_stop_button,
            const char *topic_emergency_stop = "infrastructure/emergency_stop");

    static void trigger_emergency_stop(bool toggled);
};

#endif // __INFRASTRUCTURE_H