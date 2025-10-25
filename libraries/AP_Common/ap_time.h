#pragma once

#include <sys/types.h>

#if defined(__GNUC__) || defined(__clang__)
  #include_next <time.h>
#else
  #include <time.h>
#endif

// replacement for mktime()
time_t ap_mktime(const struct tm *t);
