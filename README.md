# fixedpoint-control

[![CI](https://github.com/ACIDBURN2501/fixedpoint-control/actions/workflows/ci.yml/badge.svg)](https://github.com/ACIDBURN2501/fixedpoint-control/actions/workflows/ci.yml)

Safety-oriented fixed-point PID controller and discrete filter library in C for
embedded systems.

## Features

- **Static memory only** — opaque runtime contexts come from fixed-size pools;
  no `malloc` / `free`.
- **Coherent `fpc_*` API** — short, project-wide prefixes keep headers,
  functions, macros, and generated files readable.
- **Explicit status handling** — APIs return `enum fpc_status`; outputs are
  written through out-parameters so a valid `0` result is never ambiguous.
- **Fixed-point arithmetic** — PID, FIR, and biquad paths use `int32_t` data
  with `int64_t` intermediates for deterministic embedded execution.
- **Deterministic bounds** — pool capacity and FIR order are compile-time
  bounded, and all loops execute within known limits.
- **PID runtime controls** — independent integral clamping, derivative
  smoothing, and manual/automatic mode switching.
- **Compliance-aware design goals** — small auditable codebase with static
  allocation, explicit contracts, and unit-test coverage. Formal compliance
  evidence is not included in this repository.

## Installation

### Meson subproject

Add this repository as a subproject or wrap. The top-level project exports a
dependency object with the correct include paths for the public headers and the
generated configuration header.

```meson
fpc_dep = dependency(
  'fixedpoint-control',
  fallback : ['fixedpoint-control', 'fpc_dep']
)
```

### Installed dependency

The project installs a static library, public headers, generated configuration
headers, and a pkg-config file. After installation, consumers can use:

```meson
fpc_dep = dependency('fixedpoint-control')
```

## Requirements

- C11 compiler
- Meson and Ninja
- `pkg-config` for installed dependency discovery
- Network access on first configure when Meson fetches the wrapped
  `pool-allocator` dependency

## Runtime Lifecycle

- Call `fpc_pid_pool_init()` before creating PID controllers.
- Call `fpc_filter_pool_init()` before creating FIR or biquad filters.
- PID, FIR, and biquad objects use separate fixed-size pools.
- `FPC_MAX_INSTANCES` applies per pool type, not as one shared global limit.
- Pointers returned by `*_init()` become invalid after `*_deinit()`.

## Quick Start

### PID controller

```c
#include <stdint.h>
#include "fpc_pid.h"

int main(void)
{
        struct fpc_pid        *pid = NULL;
        struct fpc_pid_config  cfg = {
                .kp = 1000,
                .ki = 500,
                .kd = 200,
                .dt = 1000,
                .out_min = -100,
                .out_max = 100,
                .integral_min = -50,
                .integral_max = 50,
                .d_filter_alpha = 65536U
        };
        int32_t                output = 0;

        if (fpc_pid_pool_init() != FPC_STATUS_OK) {
                return 1;
        }

        if (fpc_pid_init(&pid, &cfg) != FPC_STATUS_OK) {
                return 1;
        }

        if (fpc_pid_compute(pid, 1000, 900, &output) < FPC_STATUS_OK) {
                (void)fpc_pid_deinit(pid);
                return 1;
        }

        (void)fpc_pid_deinit(pid);
        return 0;
}
```

### FIR filter

```c
#include <stdint.h>
#include "fpc_filters.h"

int main(void)
{
        static const int32_t coeffs[] = {21845, 21845, 21845};
        struct fpc_fir        *fir = NULL;
        struct fpc_fir_config  cfg = {
                .order = 3U,
                .coeffs = coeffs
        };
        int32_t                output = 0;

        if (fpc_filter_pool_init() != FPC_STATUS_OK) {
                return 1;
        }

        if (fpc_fir_init(&fir, &cfg) != FPC_STATUS_OK) {
                return 1;
        }

        if (fpc_fir_process(fir, 1000, &output) != FPC_STATUS_OK) {
                (void)fpc_fir_deinit(fir);
                return 1;
        }

        (void)fpc_fir_deinit(fir);
        return 0;
}
```

### Biquad filter

```c
#include <stdint.h>
#include "fpc_filters.h"

int main(void)
{
        struct fpc_biquad        *biquad = NULL;
        struct fpc_biquad_config  cfg = {
                .b0 = 65536,
                .b1 = 0,
                .b2 = 0,
                .a1 = 0,
                .a2 = 0
        };
        int32_t                   output = 0;

        if (fpc_filter_pool_init() != FPC_STATUS_OK) {
                return 1;
        }

        if (fpc_biquad_init(&biquad, &cfg) != FPC_STATUS_OK) {
                return 1;
        }

        if (fpc_biquad_process(biquad, 1000, &output) != FPC_STATUS_OK) {
                (void)fpc_biquad_deinit(biquad);
                return 1;
        }

        (void)fpc_biquad_deinit(biquad);
        return 0;
}
```

## Configuration

Configuration is resolved at compile time through a single header,
`include/fpc_config.h`, which checks three sources in priority order:

1. **`-DFPC_CONF_PATH="path/to/fpc_conf.h"`** — supply an explicit path on the
   compiler command line.
2. **`fpc_conf.h` on the include path** — place the file so
   `#include "fpc_conf.h"` resolves (works with any build system or no build
   system at all).
3. **Built-in defaults** — if neither of the above is present, sensible defaults
   are compiled in (see table below).

Any value can also be overridden individually with a `-D` flag regardless of
which source is used, because every define is `#ifndef`-guarded.

### Configurable values

| Macro | Description | Default |
|---|---|---|
| `FPC_MAX_INSTANCES` | Slot count for each internal pool (PID, FIR, and biquad each get their own pool of this size). | `8` |
| `FPC_POOL_ITEM_SIZE` | Size in bytes of each pool slot. Must be ≥ the largest pooled struct and a multiple of `_Alignof(max_align_t)`. | `528` |
| `FPC_FILTER_MAX_ORDER` | Maximum FIR filter order (number of taps). | `64` |

### Meson build options

When building with Meson, options are set at configure time and the values are
written into `builddir/fpc_conf.h` automatically, no manual header editing
required.

| Option | Description | Default |
|---|---|---|
| `fpc_max_instances` | Maximum number of instances per internal pool. | `8` |
| `fpc_pool_item_size` | Size in bytes of each pool slot. Must fit the largest pooled object. | `528` |

### Drag-drop / no build system

Copy `config/fpc_conf_template.h` to a location on your include path, rename it
to `fpc_conf.h`, and change the first `#if 0` to `#if 1`. Then adjust the
values to match your target.

### Header layout

- `config/fpc_conf_template.h` — copy-and-edit starting point for users
- `config/fpc_conf.h.in` — Meson `configure_file()` template (do not edit)
- `builddir/fpc_conf.h` — generated by Meson; not source-controlled
- `include/fpc_config.h` — single public config header; include this in your code

## Building

```sh
# Configure, build, and test
meson setup builddir
meson compile -C builddir
meson test -C builddir --verbose

# Override pool geometry
meson setup builddir-custom -Dfpc_max_instances=16 -Dfpc_pool_item_size=640

# Install into a staging directory
meson install -C builddir --destdir staging
```

## CI And Static Analysis

The repository includes a basic CI workflow in `.github/workflows/ci.yml` that
configures, builds, and tests the project on GitHub Actions.

The in-repo CI currently does not enforce MISRA checks or run static-analysis
tools automatically.

Recommended additional verification for compliance-focused use:

```sh
# Example warning-focused build
meson setup builddir -Dwerror=true
meson compile -C builddir

# Example external analysis tools
cppcheck --enable=warning,style,performance,portability src include tests
clang --analyze src/pid_controller.c src/filters.c
```

## Notes

| Topic | Note |
|---|---|
| **Version header** | `fpc_version.h` is generated into the build directory from `config/fpc_version.h.in`. It should not be source-controlled. |
| **Build config header** | `fpc_conf.h` is generated into the build directory from `config/fpc_conf.h.in`. It should not be source-controlled. |
| **Thread safety** | The wrapped pool allocator dependency is used without synchronization in this repository. If you need serialized pool access, review the allocator's locking hooks and validate the integration in your target environment. |
| **Manual mode** | `fpc_pid_set_mode()` can hold a manual output and rebias the integral term when returning to auto mode. |
| **Derivative smoothing** | `d_filter_alpha` is a Q16.16 coefficient. `65536U` disables smoothing; smaller values apply stronger low-pass filtering. |
| **Caller ownership** | Discard stale pointers after `*_deinit()`. Context pointers are invalid once returned to the pool. |
| **Compliance scope** | The code is compliance-oriented, not certified. Formal MISRA/IEC-61508 evidence still requires project-specific analysis and lifecycle artifacts. |
| **License** | The repository is released under the MIT license in `LICENSE`. |
