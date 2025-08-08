#include "cixi_motor_control.h"
#include "cixi_filters.h"
#include "mc_interface.h"
#include "timeout.h"
#include <math.h>

static cixi_fir_filter_t control_filter;
static cixi_fir_filter_t feedback_filter;
static control_fsm_t     motor_fsm;

void
cixi_motor_control_init (void)
{
    // Initialize the control FSM context
    control_fsm_init(&motor_fsm);

    // Initialize FIR filters for control and feedback
    fir_filter_init(&control_filter, 16, fir_filter_update_int16);
    fir_filter_init(&feedback_filter, 8, fir_filter_update_float);
}

void
control_fsm_init (control_fsm_t *fsm)
{
    fsm->state = FSM_STATE_IDLE;
}

void
control_fsm_step (int16_t control_value)
{
    float rpm = mc_interface_get_rpm();

    // ——— 1) State transitions ———
    switch (motor_fsm.state)
    {
        case FSM_STATE_IDLE:
            if (control_value > 0)
            {
                motor_fsm.state = FSM_STATE_MOVING_FORWARD;
            }
            else if (control_value < 0)
            {
                motor_fsm.state = FSM_STATE_MOVING_BACKWARD;
            }
            break;

        case FSM_STATE_MOVING_FORWARD:
            if (rpm <= FP_ZERO_THRESHOLD_ABS && control_value < 0)
            {
                motor_fsm.state = FSM_STATE_MOVING_BACKWARD;
            }
            else if (control_value == 0)
            {
                motor_fsm.state = FSM_STATE_IDLE;
            }
            break;

        case FSM_STATE_MOVING_BACKWARD:
            if (rpm >= -FP_ZERO_THRESHOLD_ABS && control_value > 0)
            {
                motor_fsm.state = FSM_STATE_MOVING_FORWARD;
            }
            else if (control_value == 0)
            {
                motor_fsm.state = FSM_STATE_IDLE;
            }
            break;
    }

    // ——— Enforce direction lock ———
    if ((motor_fsm.state == FSM_STATE_MOVING_FORWARD && control_value < 0)
        || (motor_fsm.state == FSM_STATE_MOVING_BACKWARD && control_value > 0))
    {
        control_value = 0;
    }

    // ——— Filter & actuation ———
    float filtered = control_filter.update(&control_filter, &control_value);
    if (fabsf(filtered) > FP_ZERO_THRESHOLD_ABS)
    {
        float torque      = (filtered * TORQUE_SCALE) / GEAR_RATIO;
        float req_current = 300.0f * torque / MOTOR_TORQUE_CONSTANT;
        mc_interface_set_current(req_current);
    }
    else
    {
        stop_motor();
    }

    timeout_reset();

    return;
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