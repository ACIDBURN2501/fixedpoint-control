/**
 * @file fpc_pid.h
 * @brief Safety-oriented fixed-point PID controller API.
 */

#ifndef FPC_PID_H_
#define FPC_PID_H_

#include <stdint.h>
#include "fpc_status.h"

struct fpc_pid;

/**
 * @enum fpc_pid_mode
 * @brief PID runtime operating mode.
 */
enum fpc_pid_mode {
    FPC_PID_MODE_AUTO = 0,
    FPC_PID_MODE_MANUAL = 1
};

/**
 * @struct fpc_pid_config
 * @brief PID controller configuration.
 *
 * `d_filter_alpha` is a Q16.16 coefficient in the range 0 to 65536. A value
 * of 65536 disables derivative smoothing.
 */
struct fpc_pid_config {
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t dt;
    int32_t out_min;
    int32_t out_max;
    int32_t integral_min;
    int32_t integral_max;
    uint32_t d_filter_alpha;
};

/**
 * @struct fpc_pid_state
 * @brief Runtime PID state for diagnostics and tuning.
 */
struct fpc_pid_state {
    int32_t integral;
    int32_t prev_error;
    int32_t filtered_derivative;
    int32_t last_output;
    enum fpc_pid_mode mode;
};

enum fpc_status fpc_pid_pool_init(void);
enum fpc_status fpc_pid_init(struct fpc_pid **ctx,
                             const struct fpc_pid_config *cfg);
enum fpc_status fpc_pid_deinit(struct fpc_pid *ctx);
enum fpc_status fpc_pid_reset(struct fpc_pid *ctx);
enum fpc_status fpc_pid_set_config(struct fpc_pid *ctx,
                                   const struct fpc_pid_config *cfg);
enum fpc_status fpc_pid_get_config(const struct fpc_pid *ctx,
                                   struct fpc_pid_config *cfg);
enum fpc_status fpc_pid_set_mode(struct fpc_pid *ctx,
                                 enum fpc_pid_mode mode,
                                 int32_t manual_output);
enum fpc_status fpc_pid_compute(struct fpc_pid *ctx,
                                int32_t setpoint,
                                int32_t measurement,
                                int32_t *output);
enum fpc_status fpc_pid_get_state(const struct fpc_pid *ctx,
                                  struct fpc_pid_state *state);

#endif /* FPC_PID_H_ */
