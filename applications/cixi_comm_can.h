#ifndef CAN_CIXI_COMMS_H
#define CAN_CIXI_COMMS_H

#include <stdint.h>
#include <stdbool.h>

#define LAST_BYTE_MASK      0xFFU
#define FIRST_TWO_BITS_MASK 0xC0U
#define SHIFT_8_BITS        8U
#define SHIFT_6_BITS        6U
#define MASK_CIXI_RECV_BASE 0x7F0

#define CIXI_HEARTBEAT_TIMEOUT_MS               150U
#define CIXI_HEARTBEAT_SEND_INTERVAL_MS         45U
#define CIXI_STATUS_SEND_INTERVAL_MS            10U
#define CIXI_HEARBEAT_TIMEOUT_CHECK_INTERVAL_MS 25U
#define CIXI_BMS_INFO_SEND_INTERVAL_MS          95U
#define CIXI_BMS_STATUS_SEND_INTERVAL_MS        45U

#define APPCONF_APP_TO_USE    APP_CUSTOM
#define APPCONF_SHUTDOWN_MODE SHUTDOWN_MODE_ALWAYS_ON

/**
 * @brief Function pointer type for periodic tasks.
 *
 * Represents a callback function that takes no parameters and returns no value.
 * Such functions are invoked by the scheduler whenever their configured
 * interval elapses.
 */
typedef void (*TaskFunc)(void);

/**
 * @brief Describes a single entry in the periodic task scheduler.
 *
 * Tracks when the task last ran, how often it should run, and the function to
 * call.
 */
typedef struct
{
    systime_t last_run;    /* Timestamp of the last execution */
    uint32_t  interval_ms; /* Desired interval between runs (ms). */
    TaskFunc  callback; /* Function to invoke when the interval has elapsed. */
} PeriodicTask;

/**
 * @brief Enum for identifying base CAN IDs of known CIXI frames.
 */
typedef enum
{
    /*PERS*/
    CIXI_CAN_CMD_BASE       = 0x030,
    CIXI_CAN_STATUS_BASE    = 0x038,
    CIXI_CAN_TEMP_BASE      = 0x12C,
    CIXI_CAN_ERROR_BASE     = 0x130,
    CIXI_CAN_WARNING_BASE   = 0x137,
    CIXI_CAN_VERSION_BASE   = 0x7C6,
    CIXI_CAN_HEARTBEAT_BASE = 0x7C0,
    CIXI_CAN_PERS_HEARTBEAT = 0x780,
    /*Virtual BMS*/
    CIXI_CAN_BMS_STATUS    = 0x0D1,
    CIXI_CAN_BMS_CMD       = 0x0D0,
    CIXI_CAN_BMS_INFO      = 0x0D2,
    CIXI_CAN_BMS_TEMP      = 0x0D3,
    CIXI_CAN_BMS_HEARTBEAT = 0x788,
} CixiCanFrameIdBase;

/**
 * @brief Controller state.
 *
 * Used to inform the PERS system about the current state of the VESC
 */
typedef enum
{
    CIXI_STATE_INIT    = 0U, /**< Initial state */
    CIXI_STATE_STANDBY = 1U, /**< Standby mode: system ready but idle */
    CIXI_STATE_ACTIVE  = 2U, /**< Active state: torque command is valid */
    CIXI_STATE_ERROR   = 3U  /**< Error/faulted state */
} CixiControllerState;

/**
 * @brief Control enable state values from CIXI command frame.
 *
 * Matches values from bits 0–1 of byte 2 in the command frame.
 * Defined on bit index 22 (2 bits) in the DBC.
 */
typedef enum
{
    CIXI_ENABLE_STANDBY = 0U, /**< Standby mode: system ready but idle */
    CIXI_ENABLE_ACTIVE  = 3U  /**< Active state: torque command is valid */
} CixiEnableSignal;

/**
 * @brief Enum for multiplexed pages in the CIXI status frame.
 */
typedef enum
{
    STATUS_PAGE_1 = 0,
    STATUS_PAGE_2 = 1,
    STATUS_PAGE_3 = 2
} CixiStatusPage;

typedef enum
{
    BMSEnableSignal_ON        = 1U,
    BMSEnableSignal_OFF       = 3U,
    BMSEnableSignal_DEEPSLEEP = 12U
} BMSEnableSignal;

/**
 * @brief Decoded representation of a CIXI Propulsion Controller Status
 * frame.
 */
typedef struct
{
    int16_t control_value_in; ///< Nm
    int16_t motor_torque;     ///< Nm
    int16_t rpm_sensored;     ///< RPM (scaled)
    float   electrical_power; ///< W
    float   mechanical_power; ///< W
    float   input_voltage;    ///< V
    float   input_current;    ///< A
    float   iq;               ///< A
    uint8_t status_page;      ///< Which PAGE to send (0–2)
    uint8_t control_state;    ///< Enum: INIT, STANDBY, ACTIVE, ERROR
    bool    valid;            ///< True if frame was successfully decoded
} CixiCanData;

/**
 * @brief Battery Information frame.
 */
typedef struct
{
    float   voltage; ///< Battery voltage in mV
    float   current; ///< Battery current in mA
    uint8_t soc;     ///< State of charge in percentage (0–100)
    uint8_t soh;     ///< State of health in percentage (0–100)
    uint8_t soe;     ///< State of energy in percentage (0–100)
    bool    valid;
} CixiBatteryInfo;

/**
 * @brief Battery Status frame.
 */
typedef struct
{
    CixiControllerState state;
    bool                in_charge;
    bool                in_discharge;
    bool                charger_detected;
    bool                warning_present;
    bool                error_present;
} CixiBatteryStatus;

/**
 * @brief Battery temperature frame.
 */
typedef struct
{
    int8_t mean_temperature; ///< Mean temperature in °C
    int8_t min_temperature;  ///< Minimum temperature in °C
    int8_t max_temperature;  ///< Maximum temperature in °C
    bool   valid;            ///< True if frame was successfully decoded
} CixiBatteryTemperature;

/**
 * @brief Decoded representation of a CIXI Command frame (PERS ➜
 * Controller).
 */
typedef struct
{
    int16_t          control_value;  ///< Torque command (Nm)
    CixiEnableSignal control_enable; ///< 0 = STANDBY, 3 = ACTIVE
    BMSEnableSignal  bms_enable;
    uint8_t          control_incr; ///< Command increment
    bool             valid;        ///< Set to true if parsed successfully
} CixiCanCommand;

/**
 * @brief Decoded representation of a PERS heartbeat frame.
 */
typedef struct
{
    uint16_t  heartbeat_counter;
    systime_t heartbeat_time;
    bool      valid;
} CixiCanHeartbeat;

/**
 * @brief Return Type fo Cixi CAN operations.
 */
typedef enum
{
    CIXI_CAN_OK    = 0,
    CIXI_CAN_ERROR = 1
} CixiCanStatus;

/**
 * @brief Process an incoming CIXI CAN frame (PERS ➜ Controller).
 *
 * Supports decoding of command and heartbeat frames.
 *
 * @param can_id CAN ID of the incoming frame.
 * @param data8 Pointer to the 8-byte CAN payload.
 * @param len Payload length.
 * @param is_ext True if frame came from external controller (must be true).
 * @return bool
 */
bool process_can_cixi_frame(uint32_t can_id, uint8_t *data8, uint8_t len);

/**
 * @brief Parse a PERS heartbeat frame (CAN ID 0x780).
 *
 * @param data8 Pointer to the 8-byte CAN payload.
 * @param len Payload length (should be 2).
 * @return A `CixiCanHeartbeat` struct with decoded values and validity
 * flag.
 */
CixiCanHeartbeat parse_pers_heartbeat(uint8_t *data8, int len);

/**
 * @brief Parse a CIXI Command frame sent from PERS to the controller.
 *
 * @param can_id CAN ID of the message (should match 0x030 + X).
 * @param data8 Pointer to the 8-byte CAN payload.
 * @param len Payload length (should be 4).
 * @return A `CixiCanCommand` struct with decoded values and validity flag.
 */
CixiCanCommand parse_cixi_cmd_frame(uint32_t can_id,
                                    uint8_t *data8,
                                    uint8_t  len);

/**
 * @brief Send a heartbeat frame to the PERS system over CAN.
 *
 * This transmits a 2-byte counter that increments with each call.
 *
 * @param controller_id Controller node ID (0–3) used in CAN ID.
 * @return MSG_OK if frame was sent successfully; error code otherwise.
 */
CixiCanStatus cixi_can_send_heartbeat(uint8_t controller_id);

/**
 * @brief Registers the CIXI CAN frame reception callback with the CAN
 * communication layer.
 *
 * This function configures the CAN reception system to use the
 * `process_can_cixi_frame` handler whenever a standard identifier (SID)
 * frame is received. It is required to initialize this before decoding CIXI
 * command or heartbeat frames.
 *
 * @return CIXI_CAN_OK on successful registration, CIXI_CAN_ERROR otherwise.
 */
CixiCanStatus setup_cixi_callback(void);

/**
 * @brief Constructs and transmits a CIXI propulsion controller status frame
 * over CAN.
 *
 * Based on the specified `status_page`, this function encodes the
 * appropriate subset of telemetry signals (e.g., torque, power, voltage)
 * into a 7-byte CAN frame as defined in the CIXI DBC specification. It
 * multiplexes data according to the page index and sends it using
 * `comm_can_transmit_sid`.
 *
 * The signal values in the `CixiCanData` struct are automatically scaled
 * and encoded according to the DBC's requirements (e.g., ×0.1 or ×0.01),
 * and must be provided in SI units.
 *
 * @param controller_id ID of the propulsion controller node (valid range:
 * 0–3). This ID is used to compute the CAN ID as `0x038 + controller_id`.
 * @param status Pointer to a filled `CixiCanData` structure containing the
 * values to send.
 * @return CIXI_CAN_OK if the frame was successfully transmitted;
 * CIXI_CAN_ERROR otherwise.
 */
CixiCanStatus cixi_can_send_status(uint8_t            controller_id,
                                   const CixiCanData *status);

/**
 * @brief Generate and return a populated CIXI status data structure.
 *
 * This function creates and fills a `CixiCanData` structure with
 * representative values for testing, simulation, or demonstration purposes.
 * The values are encoded in engineering units (e.g., Nm, A, V) and are
 * suitable for transmission over the CAN bus using `cixi_can_send_status`.
 *
 * All three status pages (0–2) can be generated from this single structure:
 * - Page 0: control value, motor torque, sensored RPM
 * - Page 1: electrical and mechanical power, senseless RPM
 * - Page 2: input voltage/current and Iq
 *
 * @return A `CixiCanData` struct containing example propulsion controller
 * data.
 */
CixiCanData cixi_get_status_data(CixiControllerState controller_state);

/**
 * @brief Processes a CIXI control command and sets motor behavior accordingly.
 *
 * - If ACTIVE and control_value ≠ 0: sets motor current.
 * - If ACTIVE and control_value == 0: applies brake current.
 * - If not ACTIVE: stops the motor.
 * Always resets timeout.
 *
 * @param[in] cmd Pointer to a CixiCanCommand.
 * @return true if handled successfully, false if input is NULL.
 */
bool handle_cixi_cmd(const CixiCanCommand *cmd);

void cixi_can_init(void);

/**
 * @brief Initialize and start custom application modules.
 */
void app_custom_start(void);

/**
 * @brief Stop custom modules and block until shutdown completes.
 */
void app_custom_stop(void);

/**
 * @brief Send a BMS heartbeat CAN frame.
 * @return CIXI_CAN_OK on success, CIXI_CAN_ERROR on failure.
 */
CixiCanStatus cixi_can_send_bms_heartbeat(void);

/**
 * @brief Transmit battery info over CAN.
 * @param info  Valid pointer to CixiBatteryInfo.
 * @return CIXI_CAN_OK on success, CIXI_CAN_ERROR on NULL or failure.
 */
CixiCanStatus cixi_can_send_battery_info(const CixiBatteryInfo *info);

/**
 * @brief Transmit BMS status flags over CAN.
 * @param status  Valid pointer to CixiBatteryStatus.
 * @return CIXI_CAN_OK on success, CIXI_CAN_ERROR on NULL or failure.
 */
CixiCanStatus cixi_can_send_battery_status(const CixiBatteryStatus *status);

/**
 * @brief Get the current BMS status snapshot.
 * @return Populated CixiBatteryStatus struct.
 */
CixiBatteryStatus cixi_get_battery_status(void);

/**
 * @brief Parse a 1-byte BMS command CAN frame.
 * @param data8  Pointer to data bytes.
 * @param len    Must be 1.
 * @return CixiCanCommand with .valid set accordingly.
 */
CixiCanCommand parse_bms_cmd_frame(uint8_t *data8, int len);

/**
 * @brief Update internal BMS state from a parsed command.
 * @param cmd  Pointer to a parsed CixiCanCommand.
 * @return true on valid state transition, false otherwise.
 */
bool handle_bms_cmd(const CixiCanCommand *cmd);

#endif // CAN_CIXI_COMMS_H
