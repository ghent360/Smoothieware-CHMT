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

/*
 * Source: %mbed-os%/platform
 */

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "device.h"
#include "mbed_critical.h"
#include "mbed_error.h"
#include "mbed_error_hist.h"
#include "mbed_interface.h"
#ifdef MBED_CONF_RTOS_PRESENT
#include "rtx_os.h"
#endif

#ifndef MBED_CONF_PLATFORM_ERROR_DECODE_HTTP_URL_STR
#define MBED_CONF_PLATFORM_ERROR_DECODE_HTTP_URL_STR                          "\nFor more info, visit: https://armmbed.github.io/mbedos-error/?error=0x%08X"                   // set by library:platform
#endif

#if DEVICE_STDIO_MESSAGES
#include <stdio.h>
#endif
#ifndef __STDC_FORMAT_MACROS
#define __STDC_FORMAT_MACROS
#endif
#include <inttypes.h>

#ifndef NDEBUG
#define ERROR_REPORT(ctx, error_msg, error_filename, error_line) print_error_report(ctx, error_msg, error_filename, error_line)
static void print_error_report(const mbed_error_ctx *ctx, const char *, const char *error_filename, int error_line);
#else
#define ERROR_REPORT(ctx, error_msg, error_filename, error_line) ((void) 0)
#endif

static core_util_atomic_flag error_in_progress = CORE_UTIL_ATOMIC_FLAG_INIT;
static core_util_atomic_flag halt_in_progress = CORE_UTIL_ATOMIC_FLAG_INIT;
static int error_count = 0;
static mbed_error_ctx first_error_ctx = {0};
static mbed_error_ctx last_error_ctx = {0};
static mbed_error_hook_t error_hook = NULL;
static mbed_error_status_t handle_error(mbed_error_status_t error_status, unsigned int error_value, const char *filename, int line_number, void *caller);

//Helper function to halt the system
static MBED_NORETURN void mbed_halt_system(void)
{
    // Prevent recursion if halt is called again during halt attempt - try
    // something simple instead.
    if (core_util_atomic_flag_test_and_set(&halt_in_progress)) {
        core_util_critical_section_enter();
        __DSB();
        for (;;) {
            __WFE(); // Not WFI, as don't want to wake for pending interrupts
        }
    }

    //If in ISR context, call mbed_die directly
    if (core_util_is_isr_active() || !core_util_are_interrupts_enabled()) {
        mbed_die();
    }

    // In normal context, try orderly exit(1), which eventually calls mbed_die
    exit(1);
}

WEAK MBED_NORETURN void error(const char *format, ...)
{
    // Prevent recursion if error is called again during store+print attempt
    if (!core_util_atomic_flag_test_and_set(&error_in_progress)) {
        handle_error(MBED_ERROR_UNKNOWN, 0, NULL, 0, MBED_CALLER_ADDR());
        ERROR_REPORT(&last_error_ctx, "Fatal Run-time error", NULL, 0);

#ifndef NDEBUG
        va_list arg;
        va_start(arg, format);
        mbed_error_vprintf(format, arg);
        va_end(arg);
#endif
    }

    mbed_halt_system();
}

//Set an error status with the error handling system
static mbed_error_status_t handle_error(mbed_error_status_t error_status, unsigned int error_value, const char *filename, int line_number, void *caller)
{
    mbed_error_ctx current_error_ctx;

    //Error status should always be < 0
    if (error_status >= 0) {
        //This is a weird situation, someone called mbed_error with invalid error code.
        //We will still handle the situation but change the error code to ERROR_INVALID_ARGUMENT, atleast the context will have info on who called it
        error_status = MBED_ERROR_INVALID_ARGUMENT;
    }

    //Clear the context capturing buffer
    memset(&current_error_ctx, 0, sizeof(mbed_error_ctx));
    //Capture error information
    current_error_ctx.error_status = error_status;
    current_error_ctx.error_address = (uint32_t)caller;
    current_error_ctx.error_value = error_value;
#ifdef MBED_CONF_RTOS_PRESENT
    //Capture thread info
    osRtxThread_t *current_thread = osRtxInfo.thread.run.curr;
    current_error_ctx.thread_id = (uint32_t)current_thread;
    current_error_ctx.thread_entry_address = (uint32_t)current_thread->thread_addr;
    current_error_ctx.thread_stack_size = current_thread->stack_size;
    current_error_ctx.thread_stack_mem = (uint32_t)current_thread->stack_mem;
    current_error_ctx.thread_current_sp = (uint32_t)&current_error_ctx; // Address local variable to get a stack pointer
#endif //MBED_CONF_RTOS_PRESENT

#if MBED_CONF_PLATFORM_ERROR_FILENAME_CAPTURE_ENABLED
    //Capture filename/linenumber if provided
    //Index for tracking error_filename
    strncpy(current_error_ctx.error_filename, filename, MBED_CONF_PLATFORM_MAX_ERROR_FILENAME_LEN);
    current_error_ctx.error_line_number = line_number;
#endif

    //Prevent corruption by holding out other callers
    core_util_critical_section_enter();

    //Increment error count
    error_count++;

    //Capture the fist system error and store it
    if (error_count == 1) { //first error
        memcpy(&first_error_ctx, &current_error_ctx, sizeof(mbed_error_ctx));
    }

    //copy this error to last error
    memcpy(&last_error_ctx, &current_error_ctx, sizeof(mbed_error_ctx));

#if MBED_CONF_PLATFORM_ERROR_HIST_ENABLED
    //Log the error with error log
    mbed_error_hist_put(&current_error_ctx);
#endif

    //Call the error hook if available
    if (error_hook != NULL) {
        error_hook(&last_error_ctx);
    }

    core_util_critical_section_exit();

    return MBED_SUCCESS;
}

//Return the first error
mbed_error_status_t mbed_get_first_error(void)
{
    //return the first error recorded
    return first_error_ctx.error_status;
}

//Return the last error
mbed_error_status_t mbed_get_last_error(void)
{
    //return the last error recorded
    return last_error_ctx.error_status;
}

//Gets the current error count
int mbed_get_error_count(void)
{
    //return the current error count
    return error_count;
}

//Sets a non-fatal error
mbed_error_status_t mbed_warning(mbed_error_status_t error_status, const char *error_msg, unsigned int error_value, const char *filename, int line_number)
{
    return handle_error(error_status, error_value, filename, line_number, MBED_CALLER_ADDR());
}

//Sets a fatal error, this function is marked WEAK to be able to override this for some tests
WEAK MBED_NORETURN mbed_error_status_t mbed_error(mbed_error_status_t error_status, const char *error_msg, unsigned int error_value, const char *filename, int line_number)
{
    // Prevent recursion if error is called again during store+print attempt
    if (!core_util_atomic_flag_test_and_set(&error_in_progress)) {
        //set the error reported
        (void) handle_error(error_status, error_value, filename, line_number, MBED_CALLER_ADDR());

        //On fatal errors print the error context/report
        ERROR_REPORT(&last_error_ctx, error_msg, filename, line_number);
    }

    mbed_halt_system();
}

//Register an application defined callback with error handling
mbed_error_status_t mbed_set_error_hook(mbed_error_hook_t error_hook_in)
{
    //register the new hook/callback
    if (error_hook_in != NULL)  {
        error_hook = error_hook_in;
        return MBED_SUCCESS;
    }

    return MBED_ERROR_INVALID_ARGUMENT;
}

//Retrieve the first error context from error log
mbed_error_status_t mbed_get_first_error_info(mbed_error_ctx *error_info)
{
    memcpy(error_info, &first_error_ctx, sizeof(first_error_ctx));
    return MBED_SUCCESS;
}

//Retrieve the last error context from error log
mbed_error_status_t mbed_get_last_error_info(mbed_error_ctx *error_info)
{
    memcpy(error_info, &last_error_ctx, sizeof(mbed_error_ctx));
    return MBED_SUCCESS;
}

//Makes an mbed_error_status_t value
mbed_error_status_t mbed_make_error(mbed_error_type_t error_type, mbed_module_type_t entity, mbed_error_code_t error_code)
{
    switch (error_type) {
        case MBED_ERROR_TYPE_POSIX:
            if (error_code >= MBED_POSIX_ERROR_BASE && error_code <= MBED_SYSTEM_ERROR_BASE) {
                return -error_code;
            }
            break;

        case MBED_ERROR_TYPE_SYSTEM:
            if (error_code >= MBED_SYSTEM_ERROR_BASE && error_code <= MBED_CUSTOM_ERROR_BASE) {
                return MAKE_MBED_ERROR(MBED_ERROR_TYPE_SYSTEM, entity, error_code);
            }
            break;

        case MBED_ERROR_TYPE_CUSTOM:
            if (error_code >= MBED_CUSTOM_ERROR_BASE) {
                return MAKE_MBED_ERROR(MBED_ERROR_TYPE_CUSTOM, entity, error_code);
            }
            break;

        default:
            break;
    }

    //If we are passed incorrect values return a generic system error
    return MAKE_MBED_ERROR(MBED_ERROR_TYPE_SYSTEM, MBED_MODULE_UNKNOWN, MBED_ERROR_CODE_UNKNOWN);
}

/**
 * Clears all the last error, error count and all entries in the error log.
 * @return                      0 or MBED_SUCCESS on success.
 *
 */
mbed_error_status_t mbed_clear_all_errors(void)
{
    mbed_error_status_t status = MBED_SUCCESS;

    //Make sure we dont multiple clients resetting
    core_util_critical_section_enter();
    //Clear the error and context capturing buffer
    memset(&last_error_ctx, 0, sizeof(mbed_error_ctx));
    //reset error count to 0
    error_count = 0;
#if MBED_CONF_PLATFORM_ERROR_HIST_ENABLED
    status = mbed_error_hist_reset();
#endif
    core_util_critical_section_exit();

    return status;
}

static const char *name_or_unnamed(const char *name)
{
    return name ? name : "<unnamed>";
}

#if MBED_CONF_PLATFORM_ERROR_ALL_THREADS_INFO && defined(MBED_CONF_RTOS_PRESENT)
/* Prints info of a thread(using osRtxThread_t struct)*/
static void print_thread(const osRtxThread_t *thread)
{
    mbed_error_printf("\n%s  State: 0x%" PRIX8 " Entry: 0x%08" PRIX32 " Stack Size: 0x%08" PRIX32 " Mem: 0x%08" PRIX32 " SP: 0x%08" PRIX32, name_or_unnamed(thread->name), thread->state, thread->thread_addr, thread->stack_size, (uint32_t)thread->stack_mem, thread->sp);
}

/* Prints thread info from a list */
static void print_threads_info(const osRtxThread_t *threads)
{
    while (threads != NULL) {
        print_thread(threads);
        threads = threads->thread_next;
    }
}
#endif

#ifndef NDEBUG
static void print_error_report(const mbed_error_ctx *ctx, const char *error_msg, const char *error_filename, int error_line)
{
    int error_code = MBED_GET_ERROR_CODE(ctx->error_status);
    int error_module = MBED_GET_ERROR_MODULE(ctx->error_status);

    mbed_error_printf("\n\n++ MbedOS Error Info ++\nError Status: 0x%X Code: %d Module: %d\nError Message: ", ctx->error_status, error_code, error_module);

    switch (error_code) {
        //These are errors reported by kernel handled from mbed_rtx_handlers
        case MBED_ERROR_CODE_RTOS_EVENT:
            mbed_error_printf("Kernel Error: 0x%" PRIX32 ", ", ctx->error_value);
            break;

        case MBED_ERROR_CODE_RTOS_THREAD_EVENT:
            mbed_error_printf("Thread: 0x%" PRIX32 ", ", ctx->error_value);
            break;

        case MBED_ERROR_CODE_RTOS_MUTEX_EVENT:
            mbed_error_printf("Mutex: 0x%" PRIX32 ", ", ctx->error_value);
            break;

        case MBED_ERROR_CODE_RTOS_SEMAPHORE_EVENT:
            mbed_error_printf("Semaphore: 0x%" PRIX32 ", ", ctx->error_value);
            break;

        case MBED_ERROR_CODE_RTOS_MEMORY_POOL_EVENT:
            mbed_error_printf("MemoryPool: 0x%" PRIX32 ", ", ctx->error_value);
            break;

        case MBED_ERROR_CODE_RTOS_EVENT_FLAGS_EVENT:
            mbed_error_printf("EventFlags: 0x%" PRIX32 ", ", ctx->error_value);
            break;

        case MBED_ERROR_CODE_RTOS_TIMER_EVENT:
            mbed_error_printf("Timer: 0x%" PRIX32 ", ", ctx->error_value);
            break;

        case MBED_ERROR_CODE_RTOS_MESSAGE_QUEUE_EVENT:
            mbed_error_printf("MessageQueue: 0x%" PRIX32 ", ", ctx->error_value);
            break;

        case MBED_ERROR_CODE_ASSERTION_FAILED:
            mbed_error_printf("Assertion failed: ");
            break;

        default:
            //Nothing to do here, just print the error info down
            break;
    }
    mbed_error_puts(error_msg);
    mbed_error_printf("\nLocation: 0x%" PRIX32, ctx->error_address);

    /* We print the filename passed in, not any filename in the context. This
     * avoids the console print for mbed_error being limited to the presence
     * and length of the filename storage. Note that although the MBED_ERROR
     * macro compiles out filenames unless platform.error-filename-capture-enabled
     * is turned on, MBED_ASSERT always passes filenames, and other direct
     * users of mbed_error() may also choose to.
     */
    if (error_filename) {
        mbed_error_puts("\nFile: ");
        mbed_error_puts(error_filename);
        mbed_error_printf("+%d", error_line);
    }

    mbed_error_printf("\nError Value: 0x%" PRIX32, ctx->error_value);
#ifdef MBED_CONF_RTOS_PRESENT
    mbed_error_printf("\nCurrent Thread: %s  Id: 0x%" PRIX32 " Entry: 0x%" PRIX32 " StackSize: 0x%" PRIX32 " StackMem: 0x%" PRIX32 " SP: 0x%" PRIX32 " ",
                      name_or_unnamed(((osRtxThread_t *)ctx->thread_id)->name),
                      ctx->thread_id, ctx->thread_entry_address, ctx->thread_stack_size, ctx->thread_stack_mem, ctx->thread_current_sp);
#endif

#if MBED_CONF_PLATFORM_ERROR_ALL_THREADS_INFO && defined(MBED_CONF_RTOS_PRESENT)
    mbed_error_printf("\nNext:");
    print_thread(osRtxInfo.thread.run.next);

    mbed_error_printf("\nReady:");
    print_threads_info(osRtxInfo.thread.ready.thread_list);

    mbed_error_printf("\nWait:");
    print_threads_info(osRtxInfo.thread.wait_list);

    mbed_error_printf("\nDelay:");
    print_threads_info(osRtxInfo.thread.delay_list);
#endif
    mbed_error_printf(MBED_CONF_PLATFORM_ERROR_DECODE_HTTP_URL_STR, ctx->error_status);
    mbed_error_printf("\n-- MbedOS Error Info --\n");
}
#endif //ifndef NDEBUG

#if MBED_CONF_PLATFORM_ERROR_HIST_ENABLED
//Retrieve the error context from error log at the specified index
mbed_error_status_t mbed_get_error_hist_info(int index, mbed_error_ctx *error_info)
{
    return mbed_error_hist_get(index, error_info);
}

//Retrieve the error log count
int mbed_get_error_hist_count(void)
{
    return mbed_error_hist_get_count();
}

mbed_error_status_t mbed_save_error_hist(const char *path)
{
    mbed_error_status_t ret = MBED_SUCCESS;
    mbed_error_ctx ctx = {0};
    int log_count = mbed_error_hist_get_count();
    FILE *error_log_file = NULL;

    //Ensure path is valid
    if (path == NULL) {
        ret = MBED_MAKE_ERROR(MBED_MODULE_PLATFORM, MBED_ERROR_CODE_INVALID_ARGUMENT);
        goto exit;
    }

    //Open the file for saving the error log info
    if ((error_log_file = fopen(path, "w")) == NULL) {
        ret = MBED_MAKE_ERROR(MBED_MODULE_PLATFORM, MBED_ERROR_CODE_OPEN_FAILED);
        goto exit;
    }

    //First store the first and last errors
    if (fprintf(error_log_file, "\nFirst Error: Status:0x%x ThreadId:0x%x Address:0x%x Value:0x%x\n",
                (unsigned int)first_error_ctx.error_status,
                (unsigned int)first_error_ctx.thread_id,
                (unsigned int)first_error_ctx.error_address,
                (unsigned int)first_error_ctx.error_value) <= 0) {
        ret = MBED_MAKE_ERROR(MBED_MODULE_PLATFORM, MBED_ERROR_CODE_WRITE_FAILED);
        goto exit;
    }

    if (fprintf(error_log_file, "\nLast Error: Status:0x%x ThreadId:0x%x Address:0x%x Value:0x%x\n",
                (unsigned int)last_error_ctx.error_status,
                (unsigned int)last_error_ctx.thread_id,
                (unsigned int)last_error_ctx.error_address,
                (unsigned int)last_error_ctx.error_value) <= 0) {
        ret = MBED_MAKE_ERROR(MBED_MODULE_PLATFORM, MBED_ERROR_CODE_WRITE_FAILED);
        goto exit;
    }

    //Update with error log info
    while (--log_count >= 0) {
        mbed_error_hist_get(log_count, &ctx);
        //first line of file will be error log count
        if (fprintf(error_log_file, "\n%d: Status:0x%x ThreadId:0x%x Address:0x%x Value:0x%x\n",
                    log_count,
                    (unsigned int)ctx.error_status,
                    (unsigned int)ctx.thread_id,
                    (unsigned int)ctx.error_address,
                    (unsigned int)ctx.error_value) <= 0) {
            ret = MBED_MAKE_ERROR(MBED_MODULE_PLATFORM, MBED_ERROR_CODE_WRITE_FAILED);
            goto exit;
        }
    }

exit:
    fclose(error_log_file);

    return ret;
}
#endif
