#include "mbed_interface.h"

void mbed_assert_internal(const char *expr, const char *file, int line) {
    mbed_die();
}