/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
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

/*
 * Source: %mbedmicro%/libraries/mbed/common
 */

#include "us_ticker_api.h"

/** Tickers events queue structure
 */
typedef struct {
    ticker_event_handler event_handler; /**< Event handler */
    ticker_event_t *head;               /**< A pointer to head */
} ticker_event_queue_t;

/** Ticker's interface structure - required API for a ticker
 */
typedef struct {
    void (*init)(void);                           /**< Init function */
    uint32_t (*read)(void);                       /**< Read function */
    void (*disable_interrupt)(void);              /**< Disable interrupt function */
    void (*clear_interrupt)(void);                /**< Clear interrupt function */
    void (*set_interrupt)(unsigned int timestamp); /**< Set interrupt function */
} ticker_interface_t;

/** Tickers data structure
 */
typedef struct {
    const ticker_interface_t *interface; /**< Ticker's interface */
    ticker_event_queue_t *queue;         /**< Ticker's events queue */
} ticker_data_t;

/** Irq handler which goes through the events to trigger events in the past.
 *
 * @param data    The ticker's data
 */
void ticker_irq_handler(const ticker_data_t *const data);
void us_ticker_init(void);
void us_ticker_set_interrupt(unsigned int timestamp);
void us_ticker_disable_interrupt(void);
void us_ticker_clear_interrupt(void);

static ticker_event_queue_t events;

static const ticker_interface_t us_interface = {
    .init = us_ticker_init,
    .read = us_ticker_read,
    .disable_interrupt = us_ticker_disable_interrupt,
    .clear_interrupt = us_ticker_clear_interrupt,
    .set_interrupt = us_ticker_set_interrupt,
};

static const ticker_data_t us_data = {
    .interface = &us_interface,
    .queue = &events,
};

const ticker_data_t* get_us_ticker_data(void)
{
    return &us_data;
}

void us_ticker_irq_handler(void)
{
    ticker_irq_handler(&us_data);
}
