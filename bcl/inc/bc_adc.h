#ifndef _BC_ADC_H
#define _BC_ADC_H

#include <bc_common.h>

//! @addtogroup bc_adc bc_adc
//! @brief Driver for ADC (analog to digital converter)
//! @{

//! @brief ADC channel

typedef enum
{
    //! @brief ADC channel A0
    BC_ADC_CHANNEL_A0 = 0,

    //! @brief ADC channel A1
    BC_ADC_CHANNEL_A1 = 1,

    //! @brief ADC channel A2
    BC_ADC_CHANNEL_A2 = 2,

    //! @brief ADC channel A3
    BC_ADC_CHANNEL_A3 = 3,

    //! @brief ADC channel A4
    BC_ADC_CHANNEL_A4 = 4,

    //! @brief ADC channel A5
    BC_ADC_CHANNEL_A5 = 5,

    //! @brief ADC channel VDDA
    BC_ADC_CHANNEL_VDDA = 6 // TODO add to tables

} bc_adc_channel_t;

//! @brief ADC event

typedef enum
{
    //! @brief ADC event
    BC_ADC_EVENT_DONE

} bc_adc_event_t;

//! @brief ADC burst sample rate

typedef enum
{
    //! @brief ADC burst sample rate is 20kHz
    BC_ADC_BURST_SAMPLE_RATE_20KHZ,

    //! @brief ADC burst sample rate is 10kHz
    BC_ADC_BURST_SAMPLE_RATE_10KHZ

} bc_adc_burst_sample_rate_t;

//! @brief ADC burst sample type

typedef enum
{
    //! @brief ADC burst sample type is uint8_t
    BC_ADC_BURST_SAMPLE_TYPE_U8,

    //! @brief ADC burst sample type is uint16_t
    BC_ADC_BURST_SAMPLE_TYPE_U16

} bc_adc_burst_sample_type_t;

//! @brief Initialize ADC channel
//! @param[in] channel ADC channel

void bc_adc_init(bc_adc_channel_t channel);

//! @brief Reads the ADC channel
//! @param[in] channel ADC channel
//! @param[out] result Pointer to destination where ADC conversion will be stored
//! @return true On success
//! @return false On failure

bool bc_adc_read_8b(bc_adc_channel_t channel, uint8_t *result);

//! @brief Reads the ADC channel
//! @param[in] channel ADC channel
//! @param[out] result Pointer to destination where ADC conversion will be stored
//! @return true On success
//! @return false On failure

bool bc_adc_read_16b(bc_adc_channel_t channel, uint16_t *result);

//! @brief Reads the ADC channel voltage
//! @param[in] channel ADC channel
//! @param[out] result Pointer to destination where ADC conversion will be stored
//! @return true On success
//! @return false On failure

bool bc_adc_read_voltage(bc_adc_channel_t channel, float *result);

//! @brief Set callback function
//! @param[in] channel ADC channel
//! @param[in] event_handler Function address
//! @param[in] event_param Optional event parameter (can be NULL)
//! @return true On success
//! @return false On failure

bool bc_adc_set_event_handler(bc_adc_channel_t channel, void (*event_handler)(bc_adc_channel_t, bc_adc_event_t, void *), void *event_param);

//! @brief Begins burst reading of the ADC channel
//! @param[in] channel ADC channel
//! @param[out] buffer Buffer to be filled with ADC data
//! @param[in] lenght Desired lenght of ADC data
//! @param[in] type Type of ADC data
//! @param[in] rate Sample rate

bool bc_adc_burst_read(bc_adc_channel_t channel, void *buffer, size_t length, bc_adc_burst_sample_type_t type, bc_adc_burst_sample_rate_t rate);

//! @}

#endif // _BC_ADC_H
