#include <stdint.h>
#include <string.h>
#include "app.h"
#include "ch.h"
#include "chvt.h"
#include "hal.h"
#include "cixi_comm_can.h"
#include "comm_can.h"
#include "mc_interface.h"
#include "timeout.h"

static int16_t          last_control_value = 0;
static mutex_t          cixi_mtx;
static CixiCanHeartbeat heartbeat_msg;
// Threads
static THD_FUNCTION(cixi_can_thread, arg);
static THD_WORKING_AREA(cixi_can_thread_wa, 512);

// Private variables
static volatile bool stop_now   = true;
static volatile bool is_running = false;

// Wrapper to fit into the existing application structure
void
app_custom_start (void)
{
    cixi_can_init();
}

// Wrapper to fit into the existing application structure
void
app_custom_stop (void)
{
    comm_can_set_sid_rx_callback(0);

    stop_now = true;
    while (is_running)
    {
        chThdSleepMilliseconds(1);
    }
}

// wrapper to fit into the existing application structure
void
app_custom_configure (app_configuration *conf)
{
    (void)conf; // Unused in this context
}

void
cixi_can_init (void)
{
    // intialise cixi mutex and heartbeat message
    chMtxObjectInit(&cixi_mtx);
    chMtxLock(&cixi_mtx);
    heartbeat_msg = parse_pers_heartbeat(NULL, 0);
    chMtxUnlock(&cixi_mtx);

    // Initialize CIXI CAN communication
    // comm_can_set_baud(CAN_BAUD_500K, 0);

    // Register the CIXI callback for processing CAN frames
    setup_cixi_callback();

    stop_now = false;

    // Spawn the CIXI CAN sending thread
    chThdCreateStatic(cixi_can_thread_wa,
                      sizeof(cixi_can_thread_wa),
                      NORMALPRIO,
                      cixi_can_thread,
                      NULL);
}

CixiCanStatus
setup_cixi_callback (void)
{
    comm_can_set_sid_rx_callback(process_can_cixi_frame);
    return CIXI_CAN_OK;
}

bool
process_can_cixi_frame (uint32_t can_id, uint8_t *data, uint8_t len)
{
    uint32_t       base_id;
    CixiCanCommand command_msg;

    if ((data == NULL) || (len <= 0))
    {
        return CIXI_CAN_ERROR;
    }

    base_id = can_id & (uint32_t)MASK_CIXI_RECV_BASE;

    if (base_id == (uint32_t)CIXI_CAN_CMD_BASE)
    {
        command_msg = parse_cixi_cmd_frame(can_id, data, len);

        if (command_msg.valid == true)
        {
            return handle_cixi_cmd(&command_msg);
        }
    }
    else if (can_id == (uint32_t)CIXI_CAN_PERS_HEARTBEAT)
    {
        chMtxLock(&cixi_mtx);
        systime_t heartbeat_time_prev = heartbeat_msg.heartbeat_time;
        heartbeat_msg                 = parse_pers_heartbeat(data, len);
        if (heartbeat_msg.valid == false)
        {
            // If the heartbeat is not valid, reset the time
            heartbeat_msg.heartbeat_time = heartbeat_time_prev;
        }
        chMtxUnlock(&cixi_mtx);
        return heartbeat_msg.valid;
    }

    return false;
}

CixiCanHeartbeat
parse_pers_heartbeat (uint8_t *data8, int len)
{
    CixiCanHeartbeat heartbeat;
    static bool      is_first_run = true;
    static uint16_t  last_counter = 0U;

    uint16_t current_counter = 0U;
    bool     counter_valid   = false;

    /* Initialize return struct */
    heartbeat.heartbeat_counter = 0U;
    heartbeat.valid             = false;
    heartbeat.heartbeat_time    = chVTGetSystemTimeX();

    /* Input validation */
    if ((data8 == NULL) || (len != 2))
    {
        return heartbeat;
    }

    /* Decode little-endian 16-bit counter */
    current_counter = (uint16_t)data8[0] | ((uint16_t)data8[1] << SHIFT_8_BITS);

    /* Accept first message unconditionally */
    if (is_first_run == true)
    {
        last_counter = current_counter;
        is_first_run = false;

        heartbeat.heartbeat_counter = current_counter;
        heartbeat.valid             = true;

        return heartbeat;
    }

    /* Wraparound-safe increment check (must equal last + 1) */
    counter_valid = ((uint16_t)(last_counter + 1U) == current_counter);

    if (counter_valid == true)
    {
        heartbeat.heartbeat_counter = current_counter;
        heartbeat.valid             = true;
    }
    last_counter = current_counter;

    return heartbeat;
}

CixiCanCommand
parse_cixi_cmd_frame (uint32_t can_id, uint8_t *data8, uint8_t len)
{
    (void)can_id; // can_id is not used in this function
    CixiCanCommand result;

    /* Initialize result as invalid */
    (void)memset(&result, 0, sizeof(result));
    result.valid = false;

    /* Sanity check: minimum length */
    if (data8 == NULL || len != 4U)
    {
        return result;
    }

    /* Decode control value: signed 16-bit, scaled by 0.1 */
    int16_t torque_raw
        = (int16_t)((uint16_t)data8[1] << SHIFT_8_BITS | (uint16_t)data8[0]);
    last_control_value   = torque_raw;
    result.control_value = (float)torque_raw * TORQUE_SCALE;

    /* Decode controlEnable (2 bits at bit offset 22) */
    uint8_t enable_bits = data8[2] & LAST_TWO_BITS_MASK;
    if (enable_bits > 3U)
    {
        return result;
    }
    result.control_enable = (CixiControlEnable)enable_bits;

    /* Decode controlIncr */
    result.control_incr = data8[3];

    /* Only accept if controlEnable is ACTIVE (3) */
    if (result.control_enable == CIXI_ENABLE_ACTIVE)
    {
        result.valid = true;
    }

    return result;
}

bool
handle_cixi_cmd (const CixiCanCommand *cmd)
{
    if (cmd == NULL)
    {
        return CIXI_CAN_ERROR;
    }

    if (cmd->control_enable == CIXI_ENABLE_ACTIVE)
    {
        if (cmd->control_value != 0.0F)
        {
            float speed = 10000;
            mc_interface_set_pid_speed(speed);
            timeout_reset();
        }
        else
        {
            mc_interface_set_brake_current(BRAKE_CURRENT);
            timeout_reset();
        }
    }
    else
    {
        mc_interface_set_current(STOP_CURRENT);
        timeout_reset();
    }

    return true;
}

CixiCanStatus
cixi_can_send_heartbeat (uint8_t controller_id)
{
    static uint16_t heartbeat_counter = 0U;

    uint8_t  can_payload[2];
    uint32_t can_id = 0U;
    msg_t    status = MSG_OK;

    /* Encode counter as little-endian */
    can_payload[0] = (uint8_t)(heartbeat_counter & LAST_BYTE_MASK);
    can_payload[1]
        = (uint8_t)((heartbeat_counter >> SHIFT_8_BITS) & LAST_BYTE_MASK);

    /* Construct CAN ID: base (0x7C0) + controller ID */
    can_id = (uint32_t)CIXI_CAN_HEARTBEAT_BASE + (uint32_t)controller_id;

    /* Transmit CAN frame */
    status = comm_can_transmit_sid(can_id, can_payload, 2U);

    /* Increment counter for next heartbeat */
    heartbeat_counter++;

    if (status == MSG_OK)
    {
        return CIXI_CAN_OK;
    }
    return CIXI_CAN_ERROR;
}

CixiCanData
cixi_get_status_data (CixiControlEnable control_enable)
{
    float input_current = mc_interface_read_reset_avg_input_current();
    float input_voltage = mc_interface_get_input_voltage_filtered();
    float input_power   = input_current * input_voltage;

    CixiCanData status
        = { .control_value_in = last_control_value,
            .motor_torque     = mc_interface_read_reset_avg_motor_current()
                            * MOTOR_TORQUE_CONSTANT,
            .rpm_sensored     = mc_interface_get_rpm(),
            .electrical_power = input_power,
            .mechanical_power = input_power,   // assume 100% efficiency
            .input_voltage    = input_voltage, // V
            .input_current    = input_current, // A
            .iq               = mc_interface_read_reset_avg_iq(), // A
            .status_page      = 0, // Default (not used anymore in tx)
            .control_state    = control_enable,
            .valid            = true };

    return status;
}

CixiCanStatus
cixi_can_send_status (uint8_t controller_id, const CixiCanData *status)
{
    if (status == NULL || controller_id > 3)
    {
        return CIXI_CAN_ERROR;
    }

    uint8_t  data[7];
    uint32_t base_id = CIXI_CAN_STATUS_BASE + controller_id;

    for (uint8_t page = 0; page <= 2; page++)
    {
        memset(data, 0, sizeof(data));

        // Byte 0: StatusPage (bits 0–1), NbPages (bits 2–3), ControlState (bits
        // 6–7)
        data[0] |= (page & 0x03);
        data[0] |= (3U << 2); // NbPages = 3
        data[0] |= ((status->control_state & 0x03) << 6);

        switch (page)
        {
            case STATUS_PAGE_1: {
                int16_t control_val
                    = (int16_t)(status->control_value_in / TORQUE_SCALE);
                int16_t motor_torque
                    = (int16_t)(status->motor_torque / TORQUE_SCALE);
                int16_t rpm = (int16_t)(status->rpm_sensored / RPM_SCALE);

                data[1] = (uint8_t)(control_val & LAST_BYTE_MASK);
                data[2]
                    = (uint8_t)((control_val >> SHIFT_8_BITS) & LAST_BYTE_MASK);

                data[3] = (uint8_t)(motor_torque & LAST_BYTE_MASK);
                data[4] = (uint8_t)((motor_torque >> SHIFT_8_BITS)
                                    & LAST_BYTE_MASK);

                data[5] = (uint8_t)(rpm & LAST_BYTE_MASK);
                data[6] = (uint8_t)((rpm >> SHIFT_8_BITS) & LAST_BYTE_MASK);
                break;
            }
            case STATUS_PAGE_2: {
                int16_t elec_power = (int16_t)status->electrical_power;
                int16_t mech_power = (int16_t)status->mechanical_power;
                int16_t rpm_sensless
                    = (int16_t)(status->rpm_sensored / RPM_SCALE);

                data[1] = (uint8_t)(elec_power & LAST_BYTE_MASK);
                data[2]
                    = (uint8_t)((elec_power >> SHIFT_8_BITS) & LAST_BYTE_MASK);

                data[3] = (uint8_t)(mech_power & LAST_BYTE_MASK);
                data[4]
                    = (uint8_t)((mech_power >> SHIFT_8_BITS) & LAST_BYTE_MASK);

                data[5] = (uint8_t)(rpm_sensless & LAST_BYTE_MASK);
                data[6] = (uint8_t)((rpm_sensless >> SHIFT_8_BITS)
                                    & LAST_BYTE_MASK);
                break;
            }
            case STATUS_PAGE_3: {
                uint16_t voltage
                    = (uint16_t)(status->input_voltage / EE_SIG_SCALE);
                int16_t current
                    = (int16_t)(status->input_current / EE_SIG_SCALE);
                int16_t iq = (int16_t)(status->iq / EE_SIG_SCALE);

                data[1] = (uint8_t)(voltage & LAST_BYTE_MASK);
                data[2] = (uint8_t)((voltage >> SHIFT_8_BITS) & LAST_BYTE_MASK);

                data[3] = (uint8_t)(current & LAST_BYTE_MASK);
                data[4] = (uint8_t)((current >> SHIFT_8_BITS) & LAST_BYTE_MASK);

                data[5] = (uint8_t)(iq & LAST_BYTE_MASK);
                data[6] = (uint8_t)((iq >> SHIFT_8_BITS) & LAST_BYTE_MASK);
                break;
            }
        }

        if (comm_can_transmit_sid(base_id, data, sizeof(data)) != MSG_OK)
        {
            return CIXI_CAN_ERROR;
        }
    }

    return CIXI_CAN_OK;
}

static void
check_heartbeat_timeout (void)
{
    systime_t last;

    chMtxLock(&cixi_mtx);
    last = heartbeat_msg.heartbeat_time;
    chMtxUnlock(&cixi_mtx);

    // Use ChibiOS time difference and conversion to milliseconds
    systime_t diff = chVTTimeElapsedSinceX(last); // Handles wrap-around safely
    uint32_t  diff_ms = ST2MS(diff); // Converts ticks to milliseconds

    if (diff_ms > CIXI_HEARTBEAT_TIMEOUT_MS)
    {
        // Timeout: stop the motor safely
        mc_interface_set_current(STOP_CURRENT);
        timeout_reset();
    }
}

static THD_FUNCTION(cixi_can_thread, arg)
{
    (void)arg;

    chRegSetThreadName("CIXI CAN process");
    is_running = true;

    // init comms and put pers and controller into active modes
    // for now just listen to commands for testing
    for (;;)
    {
        // Check if it is time to stop.
        if (stop_now)
        {
            is_running = false;
            return;
        }

        // check_heartbeat_timeout();
        cixi_can_send_heartbeat(0);
        CixiCanData status = cixi_get_status_data(CIXI_ENABLE_ACTIVE);
        cixi_can_send_status(0, &status);
        chThdSleepMilliseconds(20);
    }
}