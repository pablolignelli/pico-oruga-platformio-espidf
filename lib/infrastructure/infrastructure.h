#include "soc/gpio_num.h"
#ifndef __INFRASTRUCTURE_H
#define __INFRASTRUCTURE_H

class Infrastructure {
public:
    /** Called whenever the emergency stop state changes.
     * @param stopped true if the emergency stop is engaged, false if released. */
    using EmergencyStopCallback = void (*)(bool stopped);

    /**
     * @param emergency_stop_button GPIO pin the emergency stop button/switch is wired to.
     * @param on_emergency_stop Callback invoked whenever the emergency stop state changes.
     * @param topic_emergency_stop Topic used to publish the emergency stop state.
     *
     * @pre on_emergency_stop != nullptr
     *
     * @return true on success, false if the GPIO/ISR setup failed.
     */
    static bool setup(gpio_num_t emergency_stop_button, EmergencyStopCallback on_emergency_stop,
            const char *topic_emergency_stop = "infrastructure/emergency_stop");

    /**
     * Notifies of an emergency stop state change: runs the emergency stop callback
     * and publishes the new state on the emergency stop topic.
     * @param stopped true if the emergency stop is engaged, false if released.
     */
    static void trigger_emergency_stop(bool stopped);
};

#endif // __INFRASTRUCTURE_H