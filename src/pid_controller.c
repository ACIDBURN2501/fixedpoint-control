/* Configure pool size before including pool.h. */
#include "../include/fpc_config.h"
#include "../include/fpc_pool_config.h"

#include "../include/fpc_pid.h"
#include <stdbool.h>
#include <limits.h>
#include "pool.h"

struct fpc_pid {
    pool_id_t pool_id;
    uint8_t initialized;
    uint8_t mode;
    uint16_t reserved;
    int32_t manual_output;
    int32_t kp;
    int32_t ki;
    int32_t kd;
    int32_t dt;
    int32_t out_min;
    int32_t out_max;
    int32_t integral_min;
    int32_t integral_max;
    uint32_t d_filter_alpha;
    int32_t integral;
    int32_t prev_error;
    int32_t filtered_derivative;
    int32_t last_output;
};

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(struct fpc_pid) <= POOL_ITEM_SIZE,
               "POOL_ITEM_SIZE too small for struct fpc_pid");
#endif

static struct pool_t fpc_pid_pool_storage = {0};
static pool_handle_t fpc_pid_pool_handle = NULL;

static int32_t
fpc_clamp_int32(int64_t value, int32_t min, int32_t max, bool *clamped)
{
    int64_t bounded_value = value;

    if (bounded_value < (int64_t)min) {
        bounded_value = (int64_t)min;
        *clamped = true;
    } else if (bounded_value > (int64_t)max) {
        bounded_value = (int64_t)max;
        *clamped = true;
    }

    return (int32_t)bounded_value;
}

static bool
fpc_pid_config_is_valid(const struct fpc_pid_config *cfg)
{
    if (cfg == NULL) {
        return false;
    }

    if (cfg->dt <= 0) {
        return false;
    }

    if (cfg->out_min > cfg->out_max) {
        return false;
    }

    if (cfg->integral_min > cfg->integral_max) {
        return false;
    }

    if (cfg->d_filter_alpha > 65536U) {
        return false;
    }

    return true;
}

static enum fpc_status
fpc_pid_get_valid_pointer(struct fpc_pid *ctx, struct fpc_pid **valid_ctx)
{
    void *pool_ptr = NULL;

    if ((ctx == NULL) || (valid_ctx == NULL)) {
        return FPC_STATUS_NULL_PTR;
    }

    if (fpc_pid_pool_handle == NULL) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    if (ctx->initialized == 0U) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    pool_ptr = pool_get_pointer(fpc_pid_pool_handle, ctx->pool_id);
    if (pool_ptr != (void *)ctx) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    *valid_ctx = ctx;
    return FPC_STATUS_OK;
}

static enum fpc_status
fpc_pid_get_valid_const_pointer(const struct fpc_pid *ctx,
                                const struct fpc_pid **valid_ctx)
{
    void *pool_ptr = NULL;

    if ((ctx == NULL) || (valid_ctx == NULL)) {
        return FPC_STATUS_NULL_PTR;
    }

    if (fpc_pid_pool_handle == NULL) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    if (ctx->initialized == 0U) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    pool_ptr = pool_get_pointer(fpc_pid_pool_handle, ctx->pool_id);
    if (pool_ptr != (void *)ctx) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    *valid_ctx = ctx;
    return FPC_STATUS_OK;
}

static void
fpc_pid_copy_config(struct fpc_pid *ctx, const struct fpc_pid_config *cfg)
{
    ctx->kp = cfg->kp;
    ctx->ki = cfg->ki;
    ctx->kd = cfg->kd;
    ctx->dt = cfg->dt;
    ctx->out_min = cfg->out_min;
    ctx->out_max = cfg->out_max;
    ctx->integral_min = cfg->integral_min;
    ctx->integral_max = cfg->integral_max;
    ctx->d_filter_alpha = cfg->d_filter_alpha;
}

static enum fpc_status
fpc_pid_set_manual_output(struct fpc_pid *ctx, int32_t manual_output)
{
    bool clamped = false;

    ctx->manual_output = fpc_clamp_int32((int64_t)manual_output,
                                         ctx->out_min,
                                         ctx->out_max,
                                         &clamped);
    ctx->last_output = ctx->manual_output;
    return clamped ? FPC_STATUS_SATURATED : FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_pool_init(void)
{
    if (pool_init(&fpc_pid_pool_storage) != POOL_OK) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    fpc_pid_pool_handle = &fpc_pid_pool_storage;
    return FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_init(struct fpc_pid **ctx, const struct fpc_pid_config *cfg)
{
    pool_id_t pool_id;
    void *pool_ptr = NULL;
    struct fpc_pid *pid = NULL;
    pool_status_t pool_status;

    if (ctx == NULL) {
        return FPC_STATUS_NULL_PTR;
    }

    *ctx = NULL;

    if (!fpc_pid_config_is_valid(cfg)) {
        return FPC_STATUS_INVALID_PARAM;
    }

    if (fpc_pid_pool_handle == NULL) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    pool_status = pool_acquire(fpc_pid_pool_handle, &pool_id);
    if (pool_status == POOL_ERR_FULL) {
        return FPC_STATUS_POOL_FULL;
    }

    if (pool_status != POOL_OK) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    pool_status = pool_get_pointer_checked(fpc_pid_pool_handle, pool_id, &pool_ptr);
    if (pool_status != POOL_OK) {
        (void)pool_release(fpc_pid_pool_handle, pool_id);
        return FPC_STATUS_NOT_INITIALIZED;
    }

    pid = (struct fpc_pid *)pool_ptr;
    pid->pool_id = pool_id;
    pid->initialized = 1U;
    pid->mode = (uint8_t)FPC_PID_MODE_AUTO;
    pid->manual_output = 0;
    fpc_pid_copy_config(pid, cfg);
    pid->integral = 0;
    pid->prev_error = 0;
    pid->filtered_derivative = 0;
    pid->last_output = 0;

    *ctx = pid;
    return FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_deinit(struct fpc_pid *ctx)
{
    struct fpc_pid *valid_ctx = NULL;
    enum fpc_status status;

    status = fpc_pid_get_valid_pointer(ctx, &valid_ctx);
    if (status != FPC_STATUS_OK) {
        return status;
    }

    if (pool_release(fpc_pid_pool_handle, valid_ctx->pool_id) != POOL_OK) {
        return FPC_STATUS_NOT_INITIALIZED;
    }

    return FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_reset(struct fpc_pid *ctx)
{
    struct fpc_pid *valid_ctx = NULL;
    enum fpc_status status;

    status = fpc_pid_get_valid_pointer(ctx, &valid_ctx);
    if (status != FPC_STATUS_OK) {
        return status;
    }

    valid_ctx->integral = 0;
    valid_ctx->prev_error = 0;
    valid_ctx->filtered_derivative = 0;
    valid_ctx->last_output =
        ((enum fpc_pid_mode)valid_ctx->mode == FPC_PID_MODE_MANUAL) ?
        valid_ctx->manual_output : 0;
    return FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_set_config(struct fpc_pid *ctx, const struct fpc_pid_config *cfg)
{
    struct fpc_pid *valid_ctx = NULL;
    enum fpc_status status;
    bool clamped = false;

    if (!fpc_pid_config_is_valid(cfg)) {
        return FPC_STATUS_INVALID_PARAM;
    }

    status = fpc_pid_get_valid_pointer(ctx, &valid_ctx);
    if (status != FPC_STATUS_OK) {
        return status;
    }

    fpc_pid_copy_config(valid_ctx, cfg);
    valid_ctx->integral = fpc_clamp_int32((int64_t)valid_ctx->integral,
                                          valid_ctx->integral_min,
                                          valid_ctx->integral_max,
                                          &clamped);
    valid_ctx->last_output = fpc_clamp_int32((int64_t)valid_ctx->last_output,
                                             valid_ctx->out_min,
                                             valid_ctx->out_max,
                                             &clamped);

    if ((enum fpc_pid_mode)valid_ctx->mode == FPC_PID_MODE_MANUAL) {
        if (fpc_pid_set_manual_output(valid_ctx, valid_ctx->manual_output) ==
            FPC_STATUS_SATURATED) {
            clamped = true;
        }
    }

    return clamped ? FPC_STATUS_SATURATED : FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_get_config(const struct fpc_pid *ctx, struct fpc_pid_config *cfg)
{
    const struct fpc_pid *valid_ctx = NULL;
    enum fpc_status status;

    if (cfg == NULL) {
        return FPC_STATUS_NULL_PTR;
    }

    status = fpc_pid_get_valid_const_pointer(ctx, &valid_ctx);
    if (status != FPC_STATUS_OK) {
        return status;
    }

    cfg->kp = valid_ctx->kp;
    cfg->ki = valid_ctx->ki;
    cfg->kd = valid_ctx->kd;
    cfg->dt = valid_ctx->dt;
    cfg->out_min = valid_ctx->out_min;
    cfg->out_max = valid_ctx->out_max;
    cfg->integral_min = valid_ctx->integral_min;
    cfg->integral_max = valid_ctx->integral_max;
    cfg->d_filter_alpha = valid_ctx->d_filter_alpha;
    return FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_set_mode(struct fpc_pid *ctx,
                 enum fpc_pid_mode mode,
                 int32_t manual_output)
{
    struct fpc_pid *valid_ctx = NULL;
    enum fpc_status status;
    bool clamped = false;
    int64_t proportional64;
    int64_t rebias64;

    if ((mode != FPC_PID_MODE_AUTO) && (mode != FPC_PID_MODE_MANUAL)) {
        return FPC_STATUS_INVALID_PARAM;
    }

    status = fpc_pid_get_valid_pointer(ctx, &valid_ctx);
    if (status != FPC_STATUS_OK) {
        return status;
    }

    if (mode == FPC_PID_MODE_MANUAL) {
        valid_ctx->mode = (uint8_t)FPC_PID_MODE_MANUAL;
        return fpc_pid_set_manual_output(valid_ctx, manual_output);
    }

    if ((enum fpc_pid_mode)valid_ctx->mode == FPC_PID_MODE_MANUAL) {
        proportional64 = ((int64_t)valid_ctx->kp * (int64_t)valid_ctx->prev_error) /
                         (int64_t)valid_ctx->dt;
        rebias64 = (int64_t)valid_ctx->last_output -
                   proportional64 -
                   (int64_t)valid_ctx->filtered_derivative;
        valid_ctx->integral = fpc_clamp_int32(rebias64,
                                              valid_ctx->integral_min,
                                              valid_ctx->integral_max,
                                              &clamped);
    }

    valid_ctx->mode = (uint8_t)FPC_PID_MODE_AUTO;
    return clamped ? FPC_STATUS_SATURATED : FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_compute(struct fpc_pid *ctx,
                int32_t setpoint,
                int32_t measurement,
                int32_t *output)
{
    struct fpc_pid *valid_ctx = NULL;
    enum fpc_status status;
    int64_t error64;
    int64_t integral_delta64;
    int64_t derivative_delta64;
    int64_t proportional64;
    int64_t raw_output64;
    int64_t filtered_derivative64;
    int64_t derivative_step64;
    int32_t error;
    int32_t error_delta;
    int32_t raw_derivative;
    bool integral_clamped = false;
    bool output_clamped = false;

    if (output == NULL) {
        return FPC_STATUS_NULL_PTR;
    }

    *output = 0;

    status = fpc_pid_get_valid_pointer(ctx, &valid_ctx);
    if (status != FPC_STATUS_OK) {
        return status;
    }

    error64 = (int64_t)setpoint - (int64_t)measurement;
    if ((error64 > (int64_t)INT32_MAX) || (error64 < (int64_t)INT32_MIN)) {
        *output = valid_ctx->last_output;
        return FPC_STATUS_OVERFLOW;
    }
    error = (int32_t)error64;

    if ((enum fpc_pid_mode)valid_ctx->mode == FPC_PID_MODE_MANUAL) {
        valid_ctx->prev_error = error;
        valid_ctx->filtered_derivative = 0;
        *output = valid_ctx->last_output;
        return FPC_STATUS_OK;
    }

    error64 = (int64_t)error - (int64_t)valid_ctx->prev_error;
    if ((error64 > (int64_t)INT32_MAX) || (error64 < (int64_t)INT32_MIN)) {
        *output = valid_ctx->last_output;
        return FPC_STATUS_OVERFLOW;
    }
    error_delta = (int32_t)error64;

    integral_delta64 = ((int64_t)valid_ctx->ki * (int64_t)error) /
                       (int64_t)valid_ctx->dt;
    derivative_delta64 = ((int64_t)valid_ctx->kd * (int64_t)error_delta) /
                         (int64_t)valid_ctx->dt;
    proportional64 = ((int64_t)valid_ctx->kp * (int64_t)error) /
                     (int64_t)valid_ctx->dt;

    if ((derivative_delta64 > (int64_t)INT32_MAX) ||
        (derivative_delta64 < (int64_t)INT32_MIN)) {
        *output = valid_ctx->last_output;
        return FPC_STATUS_OVERFLOW;
    }

    raw_derivative = (int32_t)derivative_delta64;
    derivative_step64 = ((int64_t)valid_ctx->d_filter_alpha *
                         ((int64_t)raw_derivative -
                          (int64_t)valid_ctx->filtered_derivative)) /
                        65536LL;
    filtered_derivative64 = (int64_t)valid_ctx->filtered_derivative +
                            derivative_step64;

    if ((filtered_derivative64 > (int64_t)INT32_MAX) ||
        (filtered_derivative64 < (int64_t)INT32_MIN)) {
        *output = valid_ctx->last_output;
        return FPC_STATUS_OVERFLOW;
    }

    valid_ctx->integral = fpc_clamp_int32((int64_t)valid_ctx->integral + integral_delta64,
                                          valid_ctx->integral_min,
                                          valid_ctx->integral_max,
                                          &integral_clamped);
    valid_ctx->filtered_derivative = (int32_t)filtered_derivative64;

    raw_output64 = proportional64 +
                   (int64_t)valid_ctx->integral +
                   (int64_t)valid_ctx->filtered_derivative;
    valid_ctx->last_output = fpc_clamp_int32(raw_output64,
                                             valid_ctx->out_min,
                                             valid_ctx->out_max,
                                             &output_clamped);
    valid_ctx->prev_error = error;
    *output = valid_ctx->last_output;

    return (integral_clamped || output_clamped) ?
           FPC_STATUS_SATURATED : FPC_STATUS_OK;
}

enum fpc_status
fpc_pid_get_state(const struct fpc_pid *ctx, struct fpc_pid_state *state)
{
    const struct fpc_pid *valid_ctx = NULL;
    enum fpc_status status;

    if (state == NULL) {
        return FPC_STATUS_NULL_PTR;
    }

    status = fpc_pid_get_valid_const_pointer(ctx, &valid_ctx);
    if (status != FPC_STATUS_OK) {
        return status;
    }

    state->integral = valid_ctx->integral;
    state->prev_error = valid_ctx->prev_error;
    state->filtered_derivative = valid_ctx->filtered_derivative;
    state->last_output = valid_ctx->last_output;
    state->mode = (enum fpc_pid_mode)valid_ctx->mode;
    return FPC_STATUS_OK;
}
