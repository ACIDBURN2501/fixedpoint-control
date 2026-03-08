#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include "../include/fpc_config.h"
#include "../include/fpc_pid.h"

static struct fpc_pid_config
test_config(void)
{
    struct fpc_pid_config cfg = {
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

    return cfg;
}

static void
test_pool_init_required(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();

    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_NOT_INITIALIZED);
    assert(ctx == NULL);
}

static void
test_init_and_get_config(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    struct fpc_pid_config copy;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(ctx != NULL);
    assert(fpc_pid_get_config(ctx, &copy) == FPC_STATUS_OK);
    assert(copy.kp == cfg.kp);
    assert(copy.ki == cfg.ki);
    assert(copy.kd == cfg.kd);
    assert(copy.dt == cfg.dt);
    assert(copy.out_min == cfg.out_min);
    assert(copy.out_max == cfg.out_max);
    assert(copy.integral_min == cfg.integral_min);
    assert(copy.integral_max == cfg.integral_max);
    assert(copy.d_filter_alpha == cfg.d_filter_alpha);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

static void
test_invalid_config_rejected(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();

    assert(fpc_pid_init(NULL, &cfg) == FPC_STATUS_NULL_PTR);
    assert(fpc_pid_init(&ctx, NULL) == FPC_STATUS_INVALID_PARAM);

    cfg.dt = 0;
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_INVALID_PARAM);

    cfg = test_config();
    cfg.out_min = 10;
    cfg.out_max = 0;
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_INVALID_PARAM);

    cfg = test_config();
    cfg.integral_min = 10;
    cfg.integral_max = 0;
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_INVALID_PARAM);

    cfg = test_config();
    cfg.d_filter_alpha = 65537U;
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_INVALID_PARAM);
}

static void
test_saturation_reporting(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    int32_t output = 0;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_compute(ctx, 10000, 0, &output) == FPC_STATUS_SATURATED);
    assert(output == 100);
    assert(fpc_pid_compute(ctx, -10000, 0, &output) == FPC_STATUS_SATURATED);
    assert(output == -100);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

static void
test_state_retrieval(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    struct fpc_pid_state state;
    int32_t output = 0;

    cfg.out_min = -1000;
    cfg.out_max = 1000;
    cfg.integral_min = -1000;
    cfg.integral_max = 1000;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_compute(ctx, 1000, 500, &output) == FPC_STATUS_OK);
    assert(fpc_pid_get_state(ctx, &state) == FPC_STATUS_OK);
    assert(state.integral > 0);
    assert(state.prev_error == 500);
    assert(state.filtered_derivative == 100);
    assert(state.last_output == output);
    assert(state.mode == FPC_PID_MODE_AUTO);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

static void
test_derivative_filtering(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    struct fpc_pid_state state;
    int32_t output = 0;

    cfg.kp = 0;
    cfg.ki = 0;
    cfg.kd = 1000;
    cfg.out_min = -1000;
    cfg.out_max = 1000;
    cfg.integral_min = -1000;
    cfg.integral_max = 1000;
    cfg.d_filter_alpha = 32768U;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_compute(ctx, 1000, 0, &output) == FPC_STATUS_OK);
    assert(output == 500);
    assert(fpc_pid_get_state(ctx, &state) == FPC_STATUS_OK);
    assert(state.filtered_derivative == 500);
    assert(fpc_pid_compute(ctx, 1000, 0, &output) == FPC_STATUS_OK);
    assert(output == 250);
    assert(fpc_pid_get_state(ctx, &state) == FPC_STATUS_OK);
    assert(state.filtered_derivative == 250);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

static void
test_reset_clears_runtime_state(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    struct fpc_pid_state state;
    int32_t output = 0;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_compute(ctx, 1000, 0, &output) == FPC_STATUS_SATURATED);
    assert(fpc_pid_reset(ctx) == FPC_STATUS_OK);
    assert(fpc_pid_get_state(ctx, &state) == FPC_STATUS_OK);
    assert(state.integral == 0);
    assert(state.prev_error == 0);
    assert(state.filtered_derivative == 0);
    assert(state.last_output == 0);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

static void
test_set_config_clamps_state(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    struct fpc_pid_config tighter_cfg = test_config();
    struct fpc_pid_state state;
    int32_t output = 0;

    tighter_cfg.out_min = -10;
    tighter_cfg.out_max = 10;
    tighter_cfg.integral_min = -5;
    tighter_cfg.integral_max = 5;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_compute(ctx, 1000, 0, &output) == FPC_STATUS_SATURATED);
    assert(fpc_pid_set_config(ctx, &tighter_cfg) == FPC_STATUS_SATURATED);
    assert(fpc_pid_get_state(ctx, &state) == FPC_STATUS_OK);
    assert(state.integral <= 5);
    assert(state.last_output <= 10);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

static void
test_pool_exhaustion(void)
{
    struct fpc_pid *ctx[FPC_MAX_INSTANCES] = {0};
    struct fpc_pid *extra = NULL;
    struct fpc_pid_config cfg = test_config();
    uint16_t i;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    for (i = 0U; i < FPC_MAX_INSTANCES; ++i) {
        assert(fpc_pid_init(&ctx[i], &cfg) == FPC_STATUS_OK);
    }

    assert(fpc_pid_init(&extra, &cfg) == FPC_STATUS_POOL_FULL);
    assert(extra == NULL);

    for (i = 0U; i < FPC_MAX_INSTANCES; ++i) {
        assert(fpc_pid_deinit(ctx[i]) == FPC_STATUS_OK);
    }
}

static void
test_manual_mode_and_bumpless_return(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    struct fpc_pid_state state;
    int32_t output = 0;

    cfg.out_min = -1000;
    cfg.out_max = 1000;
    cfg.integral_min = -1000;
    cfg.integral_max = 1000;
    cfg.ki = 0;
    cfg.kd = 0;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_set_mode(ctx, FPC_PID_MODE_MANUAL, 300) == FPC_STATUS_OK);
    assert(fpc_pid_compute(ctx, 1000, 900, &output) == FPC_STATUS_OK);
    assert(output == 300);
    assert(fpc_pid_get_state(ctx, &state) == FPC_STATUS_OK);
    assert(state.mode == FPC_PID_MODE_MANUAL);
    assert(state.prev_error == 100);

    assert(fpc_pid_set_mode(ctx, FPC_PID_MODE_AUTO, 0) == FPC_STATUS_OK);
    assert(fpc_pid_get_state(ctx, &state) == FPC_STATUS_OK);
    assert(state.mode == FPC_PID_MODE_AUTO);
    assert(state.integral == 200);
    assert(fpc_pid_compute(ctx, 1000, 900, &output) == FPC_STATUS_OK);
    assert(output == 300);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

static void
test_manual_mode_clamps_output(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    int32_t output = 0;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_set_mode(ctx, FPC_PID_MODE_MANUAL, 1000) == FPC_STATUS_SATURATED);
    assert(fpc_pid_compute(ctx, 0, 0, &output) == FPC_STATUS_OK);
    assert(output == 100);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

static void
test_null_and_invalid_runtime_calls(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    struct fpc_pid_config copy;
    int32_t output = 0;
    struct fpc_pid_state state;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_reset(NULL) == FPC_STATUS_NULL_PTR);
    assert(fpc_pid_get_config(NULL, &copy) == FPC_STATUS_NULL_PTR);
    assert(fpc_pid_get_config(ctx, NULL) == FPC_STATUS_NULL_PTR);
    assert(fpc_pid_set_mode(NULL, FPC_PID_MODE_MANUAL, 0) == FPC_STATUS_NULL_PTR);
    assert(fpc_pid_get_state(NULL, &state) == FPC_STATUS_NULL_PTR);
    assert(fpc_pid_get_state(ctx, NULL) == FPC_STATUS_NULL_PTR);
    assert(fpc_pid_compute(NULL, 0, 0, &output) == FPC_STATUS_NULL_PTR);
    assert(fpc_pid_compute(ctx, 0, 0, NULL) == FPC_STATUS_NULL_PTR);

    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
    assert(fpc_pid_compute(ctx, 1000, 0, &output) == FPC_STATUS_NOT_INITIALIZED);
    assert(fpc_pid_reset(ctx) == FPC_STATUS_NOT_INITIALIZED);
    assert(fpc_pid_set_mode(ctx, FPC_PID_MODE_AUTO, 0) == FPC_STATUS_NOT_INITIALIZED);
}

static void
test_overflow_detection(void)
{
    struct fpc_pid *ctx = NULL;
    struct fpc_pid_config cfg = test_config();
    int32_t output = 0;

    assert(fpc_pid_pool_init() == FPC_STATUS_OK);
    assert(fpc_pid_init(&ctx, &cfg) == FPC_STATUS_OK);
    assert(fpc_pid_compute(ctx, INT32_MAX, INT32_MIN, &output) == FPC_STATUS_OVERFLOW);
    assert(output == 0);
    assert(fpc_pid_deinit(ctx) == FPC_STATUS_OK);
}

int
main(void)
{
    test_pool_init_required();
    test_init_and_get_config();
    test_invalid_config_rejected();
    test_saturation_reporting();
    test_state_retrieval();
    test_derivative_filtering();
    test_reset_clears_runtime_state();
    test_set_config_clamps_state();
    test_pool_exhaustion();
    test_manual_mode_and_bumpless_return();
    test_manual_mode_clamps_output();
    test_null_and_invalid_runtime_calls();
    test_overflow_detection();
    return 0;
}
