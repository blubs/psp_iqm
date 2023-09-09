#ifndef LOGGING_H
#define LOGGING_H


#define LOGGING_ENABLED
#ifdef LOGGING_ENABLED

#include <stdarg.h>

// https://stackoverflow.com/a/41236383
void log_printf(const char *fmt, ...) {
    char log_file[] = "./log.txt";
    FILE *f = fopen(log_file, "a+");
    if(f == NULL) {
        return;
    }
    va_list var_args_ptr;
    va_start(var_args_ptr, fmt);
    vfprintf(f, fmt, var_args_ptr);
    va_end(var_args_ptr);
    fclose(f);
}

#else

void log_printf() {}

#endif // LOGGING_ENABLED


#endif // LOGGING_H