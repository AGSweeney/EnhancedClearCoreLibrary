// EXCERPT — source: libClearCore/inc/SysTiming.h
// EVIDENCE: E1 | symbol: _CLEARCORE_SAMPLE_RATE_HZ | lines: 42-57

#ifndef _CLEARCORE_SAMPLE_RATE_HZ
#define _CLEARCORE_SAMPLE_RATE_HZ (5000)
#endif

/**
    ClearCore sample rate, expressed in sample times (5).
**/
#define MS_TO_SAMPLES (_CLEARCORE_SAMPLE_RATE_HZ / 1000)
/**
    Number of CPU cycles per interrupt time (24,000).
**/
#define CYCLES_PER_INTERRUPT   (CPU_CLK / _CLEARCORE_SAMPLE_RATE_HZ)
/**
    ClearCore sample time, expressed in microseconds (200us).
**/
#define SAMPLE_PERIOD_MICROSECONDS (1000000UL / _CLEARCORE_SAMPLE_RATE_HZ)
