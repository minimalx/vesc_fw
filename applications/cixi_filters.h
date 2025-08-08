#ifndef CIXI_FILTERS_H
#define CIXI_FILTERS_H

#include <stdint.h>

// ==============================
// FIR Filter for Control Value
// ==============================

#define CIXI_CONTROL_FILTER_MAX_SIZE 64U // Adjustable maximum buffer size
#define CIXI_CONTROL_ZERO_SAMPLE_COUNT \
    4U // Number of consecutive zero samples to reset

/**
 * @brief Union to hold different buffer data types.
 */
typedef union
{
    float   f32[CIXI_CONTROL_FILTER_MAX_SIZE];
    int16_t i16[CIXI_CONTROL_FILTER_MAX_SIZE];
} CIXIFIRBuffer;

/**
 * @brief Forward declaration of struct for function pointer compatibility.
 */
typedef struct cixi_fir_filter_t cixi_fir_filter_t;

/**
 * @brief Function pointer type for filter update functions.
 */
typedef float (*CIXIFIRUpdateFunc)(cixi_fir_filter_t *filter, void *new_value);

/**
 * @brief Structure for parametric FIR filter for control values.
 */
struct cixi_fir_filter_t
{
    CIXIFIRBuffer buffer;     ///< Union buffer for int16_t or float
    uint8_t       size;       ///< Number of samples in the filter (window size)
    uint8_t       index;      ///< Current index in the circular buffer
    uint8_t       zero_count; ///< Counter for consecutive zero/negative values
    CIXIFIRUpdateFunc update; ///< Function pointer to filter update method
};

/**
 * @brief Initialize the FIR filter.
 *
 * @param filter Pointer to the FIR filter structure.
 * @param size Window size (number of samples); must be ≤
 * CIXI_CONTROL_FILTER_MAX_SIZE.
 */
void fir_filter_init(cixi_fir_filter_t *filter,
                     uint8_t            size,
                     CIXIFIRUpdateFunc  updater);

/**
 * @brief Update the filter with a new value and return the filtered result.
 *
 * If 'n' or more consecutive samples are zero or negative, the filter output
 * resets to zero.
 *
 * @param new_value New incoming control value.
 * @return int16_t Filtered output value.
 */
float fir_filter_update_float(cixi_fir_filter_t *filter, void *new_value);

/**
 * @brief Update the filter with a new value and return the filtered result.
 *
 * If 3 or more consecutive samples are zero or negative, the filter output
 * resets to zero.
 *
 * @param filter Pointer to the FIR filter structure.
 * @param new_value New incoming control value.
 * @return int16_t Filtered output value.
 */
float fir_filter_update_int16(cixi_fir_filter_t *filter, void *new_value);

/**
 * @brief Reset the FIR filter state.
 *
 * @param filter Pointer to the FIR filter structure.
 */
void fir_filter_reset(cixi_fir_filter_t *filter);

#endif // CIXI_FILTERS_H