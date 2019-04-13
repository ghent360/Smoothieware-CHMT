/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MBED_TIMEREVENT_H
#define MBED_TIMEREVENT_H

#ifdef VENDOR_STM
#include "ticker_api.h"
extern "C" uint32_t us_ticker_read();
extern "C" const ticker_data_t* get_us_ticker_data(void);
#else
#include "us_ticker_api.h"
#endif

namespace mbed {

/** Base abstraction for timer interrupts
*/
class TimerEvent {
public:
    TimerEvent();
    /** The handler registered with the underlying timer interrupt
     */
    static void irq(uint32_t id);

    /** Destruction removes it...
     */
    virtual ~TimerEvent();

protected:
#ifdef VENDOR_STM
    TimerEvent(const ticker_data_t *data);
#endif
    // The handler called to service the timer event of the derived class
    virtual void handler() = 0;

    // insert in to linked list
    void insert(unsigned int timestamp);

    // remove from linked list, if in it
    void remove();

    ticker_event_t event;
#ifdef VENDOR_STM    
    const ticker_data_t *_ticker_data;
#endif
};

} // namespace mbed

#endif
