/**
 * @file cixi_filters.c
 * @brief FIR filter implementation for control values with support for
 *        float and int16_t data via function pointers and union buffers.
 *
 * This file defines a flexible FIR filter structure that can operate
 * on float or int16_t input data depending on the selected update
 * function. It includes:
 * - Runtime-configurable update function via function pointer
 * - A union buffer to support different data types
 * - Zero-reset behavior based on consecutive invalid inputs
 */

#include "cixi_filters.h"
#include <string.h> // For memset

// ==============================
// Initialize the FIR filter
// ==============================
void
fir_filter_init (CIXIFIRFilter *filter, uint8_t size, CIXIFIRUpdateFunc updater)
{
    if (filter == NULL || updater == NULL || size == 0
        || size > CIXI_CONTROL_FILTER_MAX_SIZE)
    {
        return;
    }

    memset(&filter->buffer, 0, sizeof(filter->buffer));
    filter->size       = size;
    filter->index      = 0;
    filter->zero_count = 0;
    filter->update     = updater;
}

// ==============================
// Reset the filter state
// ==============================
void
fir_filter_reset (CIXIFIRFilter *filter)
{
    if (filter == NULL)
    {
        return;
    }

    memset(&filter->buffer, 0, sizeof(filter->buffer));
    filter->index      = 0;
    filter->zero_count = 0;
}

// ==============================
// Update with float input
// ==============================
float
fir_filter_update_float (CIXIFIRFilter *filter, void *new_value)
{
    if (filter == NULL || filter->size == 0)
    {
        return 0;
    }

    float value = *(float *)new_value;

    if (value <= 0.1F && value >= -0.1F)
    {
        filter->zero_count++;
    }
    else
    {
        filter->zero_count = 0;
    }

    if (filter->zero_count >= CIXI_CONTROL_ZERO_SAMPLE_COUNT)
    {
        fir_filter_reset(filter);
        return 0;
    }

    // Insert new value
    filter->buffer.f32[filter->index] = value;
    filter->index                     = (filter->index + 1) % filter->size;

    // Compute average
    float sum = 0;
    for (uint8_t i = 0; i < filter->size; i++)
    {
        sum += filter->buffer.f32[i];
    }

    return sum / (float)filter->size;
}

// ==============================
// Update with int16_t input
// ==============================
float
fir_filter_update_int16 (CIXIFIRFilter *filter, void *new_value)
{
    if (filter == NULL || filter->size == 0)
    {
        return 0;
    }

    int16_t value = *(int16_t *)new_value;

    if (value == 0)
    {
        filter->zero_count++;
    }
    else
    {
        filter->zero_count = 0;
    }

    if (filter->zero_count >= CIXI_CONTROL_ZERO_SAMPLE_COUNT)
    {
        fir_filter_reset(filter);
        return 0;
    }

    // Insert value
    filter->buffer.i16[filter->index] = value;
    filter->index                     = (filter->index + 1) % filter->size;

    // Compute average
    int32_t sum = 0;
    for (uint8_t i = 0; i < filter->size; i++)
    {
        sum += filter->buffer.i16[i];
    }

    return (float)(sum / filter->size);
}
