#ifndef CIXI_MOTOR_CONTROL_H
#define CIXI_MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define TORQUE_SCALE         0.1F
#define RPM_SCALE            0.1F
#define EE_SIG_SCALE         0.01F
#define POLE_PAIRS           5U
#define GEAR_RATIO           11U
#define MOTOR_TO_WHEEL_RATIO 55U

#define TORQUE_TO_CURRENT_UX 500.0F

#define MOTOR_KV              10.0F
#define MOTOR_TORQUE_CONSTANT 1.0F / MOTOR_KV
#define BRAKE_CURRENT         5.0F
#define STOP_CURRENT          0.0F

#define FP_ZERO_THRESHOLD_ABS 0.1F

void cixi_motor_control_init(void);

/**
 * @brief Stops the motor by applying stop or brake current based on RPM.
 */
void stop_motor(void);

/**
 * @enum control_fsm_state_t
 * @brief States for the motor control finite state machine.
 *
 * FSM_STATE_IDLE:             No movement input or motor at rest.
 * FSM_STATE_MOVING_FORWARD:   Motor is moving forward.
 * FSM_STATE_MOVING_BACKWARD:  Motor is moving backward.
 */
typedef enum
{
    FSM_STATE_IDLE,
    FSM_STATE_MOVING_FORWARD,
    FSM_STATE_MOVING_BACKWARD
} control_fsm_state_t;

/**
 * @struct control_fsm_t
 * @brief Context for the motor control finite state machine.
 *
 * @var control_fsm_t::state
 *     Current FSM state.
 * @var control_fsm_t::moving_forward
 *     Direction flag: true if forward, false if backward.
 * @var control_fsm_t::control_filter
 *     FIR filter instance for smoothing control input.
 */
typedef struct
{
    control_fsm_state_t state;
} control_fsm_t;

/**
 * @brief Initialize the control FSM context.
 *
 * Must be called before the first call to control_fsm_step().
 *
 * @param fsm Pointer to the FSM context to initialize.
 */
void control_fsm_init(control_fsm_t *fsm);

/**
 * @brief Perform one iteration of the control FSM.
 *
 * This function reads the current motor RPM, updates the FSM state based
 * on RPM and new control input, enforces direction lock, filters the input,
 * computes required motor current, and applies it. Always resets the timeout.
 *
 * @param fsm Pointer to the FSM context.
 * @param control_value Raw control input (e.g. joystick or throttle), signed
 * 16-bit. Positive values request forward motion, negative values request
 * backward motion.
 * @return none
 */
void control_fsm_step(int16_t control_value);

#endif // CIXI_MOTOR_CONTROL_H