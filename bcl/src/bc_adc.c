#include <bc_adc.h>
#include <bc_scheduler.h>
#include <bc_irq.h>
#include <bcl.h>
#include <stm32l083xx.h>

#define VREFINT_CAL_ADDR 0x1ff80078

#define BC_ADC_CHANNEL_INTERNAL_REFERENCE 7
#define BC_ADC_CHANNEL_NONE ((bc_adc_channel_t) (-1))
#define BC_ADC_CHANNEL_COUNT ((bc_adc_channel_t) 8)

typedef struct
{
    void (*event_handler)(bc_adc_channel_t, bc_adc_event_t, void *);
    void *event_param;
    uint32_t chselr;

} bc_adc_channel_config_t;

static struct
{
    bool initialized;
    bc_adc_channel_t channel_in_progress;
    uint16_t vrefint;
    bc_scheduler_task_id_t task_id;
    bc_adc_channel_config_t channel_table[8];
}
_bc_adc =
{
    .initialized = false,
    .channel_in_progress = BC_ADC_CHANNEL_NONE,
    .channel_table =
    {
        [BC_ADC_CHANNEL_A0].chselr = ADC_CHSELR_CHSEL0,
        [BC_ADC_CHANNEL_A1].chselr = ADC_CHSELR_CHSEL1,
        [BC_ADC_CHANNEL_A2].chselr = ADC_CHSELR_CHSEL2,
        [BC_ADC_CHANNEL_A3].chselr = ADC_CHSELR_CHSEL3,
        [BC_ADC_CHANNEL_A4].chselr = ADC_CHSELR_CHSEL4,
        [BC_ADC_CHANNEL_A5].chselr = ADC_CHSELR_CHSEL5,
        [BC_ADC_CHANNEL_VDDA].chselr = ADC_CHSELR_CHSEL,
        [BC_ADC_CHANNEL_INTERNAL_REFERENCE] =
        {
            NULL,
            NULL,
            ADC_CHSELR_CHSEL17
        }
    }
};

static const uint32_t _bc_adc_arr_table[2] =
{
    [0] = 800,
    [1] = 1600
};

static const uint32_t _bc_adc_ccr_table[2] =
{
    [0] = DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_TEIE,
    [1] = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_TCIE | DMA_CCR_TEIE
};

static void _bc_adc_task(void *param);

static bool _bc_adc_measure_channel(const bc_adc_channel_t channel);

static inline void _bc_adc_calibration(void);

void bc_adc_init(bc_adc_channel_t channel)
{
    if (_bc_adc.initialized != true)
    {
        bc_gpio_set_mode(channel, BC_GPIO_MODE_ANALOG);

        // Enable ADC clock
        RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

        // Errata workaround
        RCC->APB2ENR;

        // Set auto-off mode, left align
        ADC1->CFGR1 |= ADC_CFGR1_AUTOFF | ADC_CFGR1_ALIGN;

        // Enable Over-sampler with ratio (16x) and set PCLK/2 as a clock source
        ADC1->CFGR2 = ADC_CFGR2_OVSE | ADC_CFGR2_OVSR_1 | ADC_CFGR2_OVSR_0 | ADC_CFGR2_CKMODE_0;

        // Sampling time selection (3.5 cycles)
        ADC1->SMPR |= ADC_SMPR_SMP_0;

        // Enable ADC voltage regulator
        ADC1->CR |= ADC_CR_ADVREGEN;

        // Load Vrefint constant from ROM
        _bc_adc.vrefint = (*(uint16_t *) VREFINT_CAL_ADDR) << 4;

        // Enable ADC interrupt
        NVIC_SetPriority(ADC1_COMP_IRQn, 1);
        NVIC_EnableIRQ(ADC1_COMP_IRQn);

        // Enable TIM6 clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

        // The update event is selected as a trigger output (TRGO)
        TIM6->CR2 = TIM_CR2_MMS_1;

        // Enable update interrupt
        TIM6->SR = TIM_SR_UIF;

        // Enable DMA1
        RCC->AHBENR |= RCC_AHBENR_DMA1EN;

        // Enable DMA 1 channel 1 interrupt
        NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
        NVIC_EnableIRQ(DMA1_Channel1_IRQn);

        _bc_adc.initialized = true;
    }

    // TODO ERROR (re-register with every INIT call) !!!
    _bc_adc.task_id = bc_scheduler_register(_bc_adc_task, NULL, BC_TICK_INFINITY);
}

bool bc_adc_read_8b(bc_adc_channel_t channel, uint8_t *result)
{
    if (!_bc_adc_measure_channel(channel))
    {
        return false;
    }

    if (result != NULL)
    {
        *(uint8_t *) result = ADC1->DR >> 8;
    }

    return true;
}

bool bc_adc_read_16b(bc_adc_channel_t channel, uint16_t *result)
{
    if (!_bc_adc_measure_channel(channel))
    {
        return false;
    }

    if (result != NULL)
    {
        *(uint16_t *) result = ADC1->DR;
    }

    return true;
}

bool bc_adc_read_voltage(bc_adc_channel_t channel, float *result)
{
    float vdda;

    // Enable internal reference
    ADC->CCR |= ADC_CCR_VREFEN;

    if (!_bc_adc_measure_channel(BC_ADC_CHANNEL_INTERNAL_REFERENCE))
    {
        // Enable internal reference
        ADC->CCR &= ~ADC_CCR_VREFEN;

        return false;
    }

    // Enable internal reference
    ADC->CCR &= ~ADC_CCR_VREFEN;

    // Compute actual VDDA
    vdda = 3.f * ((float) _bc_adc.vrefint / (float) ADC1->DR);

    if (channel != BC_ADC_CHANNEL_VDDA)
    {
        if (_bc_adc_measure_channel(channel))
        {
            if (result != NULL)
            {
                *(float *) result = ADC1->DR * (vdda / 65536.f);
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        *result = vdda;
    }

    return true;
}

bool bc_adc_set_event_handler(bc_adc_channel_t channel, void (*event_handler)(bc_adc_channel_t, bc_adc_event_t, void *), void *event_param)
{
    // Check if ongoing on edited channel
    if (_bc_adc.channel_in_progress == channel)
    {
        return false;
    }

    _bc_adc.channel_table[channel].event_handler = event_handler;
    _bc_adc.channel_table[channel].event_param = event_param;

    return true;
}

bool bc_adc_burst_read(bc_adc_channel_t channel, void *buffer, size_t length, bc_adc_burst_sample_type_t type, bc_adc_burst_sample_rate_t rate)
{
    // If ongoing conversion...
    if (_bc_adc.channel_in_progress != BC_ADC_CHANNEL_NONE)
    {
        return false;
    }

    _bc_adc.channel_in_progress = channel;

    // Set ADC channel
    ADC1->CHSELR = _bc_adc.channel_table[channel].chselr;

    // Enable PLL and disable sleep
    bc_module_core_pll_enable();

    // Set memory incrementation, direction from peripheral, enable the transfer complete and error interrupts
    DMA1_Channel1->CCR = _bc_adc_ccr_table[type];

    // Configure request 0 selection for DMA1 Channel1
    DMA1_CSELR->CSELR &= ~DMA_CSELR_C1S;
    DMA1_CSELR->CSELR |= (uint32_t) (0 << DMA_CSELR_C1S_Pos);

    // Configure DMA channel data length
    DMA1_Channel1->CNDTR = length;

    // Configure DMA channel source address
    DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;

    // Configure DMA channel destination address
    DMA1_Channel1->CMAR = (uint32_t) buffer;

    // Enable the peripheral
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // Set auto reload register
    TIM6->ARR = _bc_adc_arr_table[rate];

    // Enable TIM6 (external trigger)
    TIM6->CR1 |= TIM_CR1_CEN;

    // Enable DMA and external trigger (TIM6)
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_EXTEN_0;

    // Start ADC measurement
    ADC1->CR |= ADC_CR_ADSTART;

    return true;
}

static void _bc_adc_task(void *param)
{
    (void) param;

    bc_adc_channel_config_t *adc = &_bc_adc.channel_table[_bc_adc.channel_in_progress];
    bc_adc_channel_t pending_result_channel;

    // Update pending channel result
    pending_result_channel = _bc_adc.channel_in_progress;

    // Release ADC for further conversion
    _bc_adc.channel_in_progress = BC_ADC_CHANNEL_NONE;

    // Perform event call-back
    adc->event_handler(pending_result_channel, BC_ADC_EVENT_DONE, adc->event_param);
}

static bool _bc_adc_measure_channel(const bc_adc_channel_t channel)
{
    // If ongoing conversion...
    if (_bc_adc.channel_in_progress != BC_ADC_CHANNEL_NONE)
    {
        return false;
    }

    // Set ADC channel
    ADC1->CHSELR = _bc_adc.channel_table[channel].chselr;

    // Disable all ADC interrupts
    ADC1->IER = 0;

    // Clear EOS flag (it is cleared by software writing 1 to it)
    ADC1->ISR = ADC_ISR_EOS;

    // Start the AD measurement
    ADC1->CR |= ADC_CR_ADSTART;

    // wait for end of measurement
    while ((ADC1->ISR & ADC_ISR_EOS) == 0)
    {
        continue;
    }

    return true;
}

static inline void _bc_adc_calibration(void)
{
    // Perform ADC calibration
    ADC1->CR |= ADC_CR_ADCAL;
    while ((ADC1->ISR & ADC_ISR_EOCAL) == 0)
    {
        continue;
    }
}

void DMA1_Channel1_IRQHandler(void)
{
    // Transfer error interrupt management
    if ((DMA1->ISR & DMA_ISR_TEIF1) != 0)
    {
        // TODO Not sure what should happen here

        // Disable the transfer error interrupt
        DMA1_Channel1->CCR &= ~DMA_CCR_TEIE;

        // Clear the transfer error flag
        DMA1->IFCR = DMA_IFCR_CTEIF1;

        bc_module_core_pll_disable();

        TIM6->CR1 &= ~TIM_CR1_CEN;

        // Transfer error callback
        // _bc_adc_burst_error_handler();
    }

    // Transfer complete interrupt management
    if ((DMA1->ISR & DMA_ISR_TCIF1) != 0)
    {
        // Disable the transfer error and complete interrupts
        DMA1_Channel1->CCR &= ~(DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_EN);

        // Disable TIM6 (external trigger)
        TIM6->CR1 &= ~TIM_CR1_CEN;

        // Clear the transfer complete flag
        DMA1->IFCR = DMA_IFCR_CTCIF1;

        bc_module_core_pll_disable();

        // Plan task that call event handler
        bc_scheduler_plan_now(_bc_adc.task_id);
    }
}
