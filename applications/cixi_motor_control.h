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

#define TORQUE_TO_CURRENT_UX    500.0F
#define MAX_MOTOR_ERPM          17250U
#define THRESHOLD_CONTROL_VALUE 1U

#define MOTOR_KV              10.0F
#define MOTOR_TORQUE_CONSTANT 1.0F / MOTOR_KV
#define BRAKE_CURRENT         5.0F
#define STOP_CURRENT          0.0F

void cixi_motor_control_init(void);

/**
 * @brief Stops the motor by applying stop or brake current based on RPM.
 */
void stop_motor(void);

/**
 * @brief Requests motor current based on control input and motion
 * direction.
 *
 * Locks motion direction based on RPM and input sign. Ignores inputs that
 * reverse direction. Applies filtering and computes current if allowed.
 * Stops motor otherwise.
 *
 * @param control_value Control input (positive or negative).
 * @return true if moving forward, false if moving backward.
 */
bool request_current_fsm(int16_t control_value);

#endif // CIXI_MOTOR_CONTROL_H