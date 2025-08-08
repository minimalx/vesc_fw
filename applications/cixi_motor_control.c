#include "cixi_motor_control.h"
#include "cixi_filters.h"
#include "mc_interface.h"
#include "timeout.h"

static CIXIFIRFilter control_filter;
static CIXIFIRFilter feedback_filter;

void
cixi_motor_control_init (void)
{
    fir_filter_init(&control_filter, 16, fir_filter_update_int16);
    fir_filter_init(&feedback_filter, 8, fir_filter_update_float);
}

void
stop_motor (void)
{
    if (mc_interface_get_rpm() < 100)
    {
        mc_interface_set_current(STOP_CURRENT);
    }
    else
    {
        // If the motor is still spinning, apply brake current
        mc_interface_set_brake_current(BRAKE_CURRENT);
    }
}

bool
request_current_fsm (int16_t control_value)
{
    static bool moving_forward = true;

    // Read the current motor RPM
    float motor_rpm = mc_interface_get_rpm();

    // Determine motion direction and update moving_forward state
    if (motor_rpm >= -0.1F && control_value > 0)
    {
        moving_forward = true;
    }
    else if (motor_rpm <= 0.1F && control_value < 0)
    {
        moving_forward = false;
    }

    // Enforce direction lock: discard opposite-direction input
    if ((moving_forward && control_value < 0)
        || (!moving_forward && control_value > 0))
    {
        control_value = 0;
    }

    // Apply FIR filtering to the control value
    float filtered_value
        = control_filter.update(&control_filter, &control_value);

    // If the filtered value is valid (non-zero in allowed direction),
    // compute current
    if (filtered_value != 0)
    {
        float req_motor_torque
            = ((float)filtered_value * TORQUE_SCALE) / (float)GEAR_RATIO;
        float req_current = 300.0F * req_motor_torque / MOTOR_TORQUE_CONSTANT;

        mc_interface_set_current(req_current);
    }
    else
    {
        stop_motor();
    }

    timeout_reset();
    return moving_forward;
}