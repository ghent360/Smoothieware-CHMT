/* mbed Library - ADC
 * Copyright (c) 2010, sblandford
 * released under MIT license http://mbed.org/licence/mit
 */

#ifndef MBED_ADC_H
#define MBED_ADC_H

#include "PinNames.h" // mbed.h lib
#define XTAL_FREQ       12000000
#define MAX_ADC_CLOCK   13000000
#define CLKS_PER_SAMPLE 64

namespace mbed {
class ADC {
public:

    //Initialize ADC with ADC maximum sample rate of
    //sample_rate and system clock divider of cclk_div
    //Maximum recommened sample rate is 184000
    ADC(int sample_rate, int cclk_div);

    //Enable/disable ADC on pin according to state
    //and also select/de-select for next conversion
    void setup(PinName pin, int state);

    //Enable/disable burst mode according to state
    void burst(int state);

    //Return burst mode enabled/disabled
    int burst(void);

    //Return startmode state according to mode_edge=0: mode and mode_edge=1: edge
    int startmode(int mode_edge);

    //Set interrupt enable/disable for pin to state
    void interrupt_state(PinName pin, int state);

    //Append custom global interrupt handler
    void append(void(*fptr)(int chan, uint32_t value));

    int _pin_to_channel(PinName pin);
private:
    int _adc_clk_freq;
    void adcisr(void);
    static void _adcisr(void);
    static ADC *instance;

    uint32_t _adc_data[8];
    void(*_adc_g_isr)(int chan, uint32_t value);
};
}

#endif
