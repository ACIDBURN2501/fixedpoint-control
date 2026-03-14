/**
 * @file fpc_conf.h
 * @brief User configuration file for fixedpoint-control.
 *
 * Copy this file next to your project's include path and rename it to
 * `fpc_conf.h`.  Set the first `#if 0` to `#if 1` to enable its content,
 * then adjust the values below to match your application.
 *
 * Three ways to provide this configuration (in priority order):
 *
 *  1. Pass `-DFPC_CONF_PATH="path/to/your/fpc_conf.h"` on the compiler
 *     command line.  The path is used verbatim inside an `#include`.
 *  2. Place an `fpc_conf.h` file so that `#include "fpc_conf.h"` resolves
 *     (e.g. next to the `fixedpoint-control/` include directory, or via
 *     a `-I` flag).
 *  3. Do nothing — sensible defaults are compiled in.
 *
 * When building with Meson, the build system generates the configuration
 * automatically from `meson_options.txt`; you do not need this file.
 */

/* clang-format off */
#if 0 /* Set this to "1" to enable content */

#ifndef FPC_CONF_H_
#define FPC_CONF_H_

/*====================
 * POOL CONFIGURATION
 *====================*/

/**
 * Maximum number of simultaneously active PID / FIR / biquad instances.
 * Each instance occupies one slot in the internal pool allocator.
 */
#define FPC_MAX_INSTANCES       8

/**
 * Size in bytes of each pool slot.
 *
 * Must be:
 *   - Large enough to hold the largest controller struct
 *     (currently `struct fpc_fir` at ~520 bytes with default order).
 *   - A multiple of `_Alignof(max_align_t)` (typically 16 on most platforms).
 *
 * The library will fail at compile time if the value is too small.
 */
#define FPC_POOL_ITEM_SIZE      528

/*====================
 * FILTER CONFIGURATION
 *====================*/

/**
 * Maximum FIR filter order (number of taps).
 * Increasing this value grows the FIR struct's internal arrays and therefore
 * also the minimum `FPC_POOL_ITEM_SIZE`.
 */
#define FPC_FILTER_MAX_ORDER    64U

#endif /* FPC_CONF_H_ */
#endif
/* clang-format on */
