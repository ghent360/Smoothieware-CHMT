/* mbed Library - ADC
 * Copyright (c) 2010, sblandford
 * released under MIT license http://mbed.org/licence/mit
 */
#include "mbed.h"
#undef ADC
#include "adc.h"

#ifdef __STM32F4__
#include "pinmap.h"
#include "PeripheralPins.h"
#include "mri.h"

#define STM_ADC ADC1
#endif

using namespace mbed;

ADC *ADC::instance;

#ifndef __STM32F4__
ADC::ADC(int sample_rate, int cclk_div){
    int i, adc_clk_freq, pclk, clock_div, max_div=1;

    //Work out CCLK
    adc_clk_freq=CLKS_PER_SAMPLE*sample_rate;
    int m = (LPC_SC->PLL0CFG & 0xFFFF) + 1;
    int n = (LPC_SC->PLL0CFG >> 16) + 1;
    int cclkdiv = LPC_SC->CCLKCFG + 1;
    int Fcco = (2 * m * XTAL_FREQ) / n;
    int cclk = Fcco / cclkdiv;

    //Power up the ADC
    LPC_SC->PCONP |= (1 << 12);
    //Set clock at cclk / 1.
    LPC_SC->PCLKSEL0 &= ~(0x3 << 24);
    switch (cclk_div) {
        case 1:
            LPC_SC->PCLKSEL0 |= 0x1 << 24;
            break;
        case 2:
            LPC_SC->PCLKSEL0 |= 0x2 << 24;
            break;
        case 4:
            LPC_SC->PCLKSEL0 |= 0x0 << 24;
            break;
        case 8:
            LPC_SC->PCLKSEL0 |= 0x3 << 24;
            break;
        default:
            printf("ADC Warning: ADC CCLK clock divider must be 1, 2, 4 or 8. %u supplied.\n",
                cclk_div);
            printf("Defaulting to 1.\n");
            LPC_SC->PCLKSEL0 |= 0x1 << 24;
            break;
    }
    pclk = cclk / cclk_div;
    clock_div=pclk / adc_clk_freq;

    if (clock_div > 0xFF) {
        printf("ADC Warning: Clock division is %u which is above 255 limit. Re-Setting at limit.\n", clock_div);
        clock_div=0xFF;
    }
    if (clock_div == 0) {
        printf("ADC Warning: Clock division is 0. Re-Setting to 1.\n");
        clock_div=1;
    }

    _adc_clk_freq=pclk / clock_div;
    if (_adc_clk_freq > MAX_ADC_CLOCK) {
        printf("ADC Warning: Actual ADC sample rate of %u which is above %u limit\n",
            _adc_clk_freq / CLKS_PER_SAMPLE, MAX_ADC_CLOCK / CLKS_PER_SAMPLE);
        while ((pclk / max_div) > MAX_ADC_CLOCK) max_div++;
        printf("ADC Warning: Maximum recommended sample rate is %u\n", (pclk / max_div) / CLKS_PER_SAMPLE);
    }

    LPC_ADC->ADCR =
        ((clock_div - 1 ) << 8 ) |    //Clkdiv
        ( 1 << 21 );                  //A/D operational

    //Default no channels enabled
    LPC_ADC->ADCR &= ~0xFF;
    //Default NULL global custom isr
    _adc_g_isr = NULL;
    //Initialize arrays
    for (i=7; i>=0; i--) {
        _adc_data[i] = 0;
    }

    //* Attach IRQ
    instance = this;
    NVIC_SetVector(ADC_IRQn, (uint32_t)&_adcisr);

    //Disable global interrupt
    LPC_ADC->ADINTEN &= ~0x100;
};

void ADC::_adcisr(void)
{
    instance->adcisr();
}

void ADC::adcisr(void)
{
    uint32_t stat;
    int chan;

    // Read status
    stat = LPC_ADC->ADSTAT;
    //Scan channels for over-run or done and update array
    if (stat & 0x0101) _adc_data[0] = LPC_ADC->ADDR0;
    if (stat & 0x0202) _adc_data[1] = LPC_ADC->ADDR1;
    if (stat & 0x0404) _adc_data[2] = LPC_ADC->ADDR2;
    if (stat & 0x0808) _adc_data[3] = LPC_ADC->ADDR3;
    if (stat & 0x1010) _adc_data[4] = LPC_ADC->ADDR4;
    if (stat & 0x2020) _adc_data[5] = LPC_ADC->ADDR5;
    if (stat & 0x4040) _adc_data[6] = LPC_ADC->ADDR6;
    if (stat & 0x8080) _adc_data[7] = LPC_ADC->ADDR7;

    // Channel that triggered interrupt
    chan = (LPC_ADC->ADGDR >> 24) & 0x07;
    //User defined interrupt handlers
    if (_adc_g_isr != NULL)
        _adc_g_isr(chan, _adc_data[chan]);
    return;
}

int ADC::_pin_to_channel(PinName pin) {
    int chan;
    switch (pin) {
        case p15://=p0.23 of LPC1768
        default:
            chan=0;
            break;
        case p16://=p0.24 of LPC1768
            chan=1;
            break;
        case p17://=p0.25 of LPC1768
            chan=2;
            break;
        case p18://=p0.26 of LPC1768
            chan=3;
            break;
        case p19://=p1.30 of LPC1768
            chan=4;
            break;
        case p20://=p1.31 of LPC1768
            chan=5;
            break;
    }
    return(chan);
}

//Enable or disable an ADC pin
void ADC::setup(PinName pin, int state) {
    int chan;
    chan=_pin_to_channel(pin);
    if ((state & 1) == 1) {
        switch(pin) {
            case p15://=p0.23 of LPC1768
            default:
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 14);
                LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 14;
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 14);
                LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 14;
                break;
            case p16://=p0.24 of LPC1768
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 16);
                LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 16;
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 16);
                LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 16;
                break;
            case p17://=p0.25 of LPC1768
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 18);
                LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 18;
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 18);
                LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 18;
                break;
            case p18://=p0.26 of LPC1768:
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 20);
                LPC_PINCON->PINSEL1 |= (unsigned int)0x1 << 20;
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 20);
                LPC_PINCON->PINMODE1 |= (unsigned int)0x2 << 20;
                break;
            case p19://=p1.30 of LPC1768
                LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 28);
                LPC_PINCON->PINSEL3 |= (unsigned int)0x3 << 28;
                LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 28);
                LPC_PINCON->PINMODE3 |= (unsigned int)0x2 << 28;
                break;
            case p20://=p1.31 of LPC1768
                LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 30);
                LPC_PINCON->PINSEL3 |= (unsigned int)0x3 << 30;
                LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 30);
                LPC_PINCON->PINMODE3 |= (unsigned int)0x2 << 30;
               break;
        }
        //Only one channel can be selected at a time if not in burst mode
        if (!burst()) LPC_ADC->ADCR &= ~0xFF;
        //Select channel
        LPC_ADC->ADCR |= (1 << chan);
    }
    else {
        switch(pin) {
            case p15://=p0.23 of LPC1768
            default:
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 14);
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 14);
                break;
            case p16://=p0.24 of LPC1768
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 16);
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 16);
                break;
            case p17://=p0.25 of LPC1768
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 18);
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 18);
                break;
            case p18://=p0.26 of LPC1768:
                LPC_PINCON->PINSEL1 &= ~((unsigned int)0x3 << 20);
                LPC_PINCON->PINMODE1 &= ~((unsigned int)0x3 << 20);
                break;
            case p19://=p1.30 of LPC1768
                LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 28);
                LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 28);
                break;
            case p20://=p1.31 of LPC1768
                LPC_PINCON->PINSEL3 &= ~((unsigned int)0x3 << 30);
                LPC_PINCON->PINMODE3 &= ~((unsigned int)0x3 << 30);
                break;
        }
        LPC_ADC->ADCR &= ~(1 << chan);
    }
}

//Enable or disable burst mode
void ADC::burst(int state) {
    if ((state & 1) == 1) {
        if (startmode(0) != 0)
            fprintf(stderr, "ADC Warning. startmode is %u. Must be 0 for burst mode.\n", startmode(0));
        LPC_ADC->ADCR |= (1 << 16);
    }
    else
        LPC_ADC->ADCR &= ~(1 << 16);
}
//Return burst mode state
int  ADC::burst(void) {
    return((LPC_ADC->ADCR & (1 << 16)) >> 16);
}

//Return startmode state according to mode_edge=0: mode and mode_edge=1: edge
int ADC::startmode(int mode_edge){
    switch (mode_edge) {
        case 0:
        default:
            return((LPC_ADC->ADCR >> 24) & 0x07);
        case 1:
            return((LPC_ADC->ADCR >> 27) & 0x01);
    }
}

//Set interrupt enable/disable for pin to state
void ADC::interrupt_state(PinName pin, int state) {
    int chan;

    chan = _pin_to_channel(pin);
    if (state == 1) {
        LPC_ADC->ADINTEN &= ~0x100;
        LPC_ADC->ADINTEN |= 1 << chan;
        /* Enable the ADC Interrupt */
        NVIC_EnableIRQ(ADC_IRQn);
    } else {
        LPC_ADC->ADINTEN &= ~( 1 << chan );
        //Disable interrrupt if no active pins left
        if ((LPC_ADC->ADINTEN & 0xFF) == 0)
            NVIC_DisableIRQ(ADC_IRQn);
    }
}

#else
// TODO(ghent360): fix the ADC
ADC::ADC(int sample_rate, int cclk_div) {
    scan_count = 0;
    scan_index = 0;

    memset(scan_chan_lut, 0xFF, sizeof(scan_chan_lut));
}

int ADC::_pin_to_channel(PinName pin) {
    uint32_t function = pinmap_function(pin, PinMap_ADC);
    uint8_t chan = 0xFF;

    if (function != (uint32_t)NC)
        chan = scan_chan_lut[STM_PIN_CHANNEL(function)];

    return chan;
}

void ADC::burst(int state) {}
void ADC::setup(PinName pin, int state) {}
void ADC::interrupt_state(PinName pin, int state) {}

void ADC::adcisr(void)
{
    uint16_t data;
    
    // must read data before checking overflow bit
    data = STM_ADC->DR; // to be sure we are valid

    if (STM_ADC->SR & ADC_SR_OVR) {
        // conversion was clobbered by overflow, clear its flag too
        STM_ADC->SR &= ~(ADC_SR_OVR | ADC_SR_EOC);

        // overrun will abort scan sequence, next start will resume from beginning
        scan_index = 0;
        __debugbreak();
    } else if (STM_ADC->SR & ADC_SR_EOC) {
        STM_ADC->SR &= ~ADC_SR_EOC;

        if (_adc_g_isr != NULL)
            _adc_g_isr(scan_index++, data);
        
        if (scan_index >= scan_count)
            scan_index = 0;
    }
}
#endif // __STM32F4__

void ADC::_adcisr(void)
{
    instance->adcisr();
}

//append global interrupt handler to function isr
void ADC::append(void(*fptr)(int chan, uint32_t value)) {
    _adc_g_isr = fptr;
}
