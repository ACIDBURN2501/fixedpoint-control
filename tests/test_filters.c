#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "../include/fpc_config.h"
#include "../include/fpc_filters.h"

typedef void (*test_fn)(void);

static const char *current_test_name = NULL;
static unsigned int current_test_index = 0U;

static void
test_assert_impl(int condition, const char *expr, const char *file, int line)
{
    if (condition) {
        return;
    }

    fprintf(stderr, "not ok %u - %s\n", current_test_index, current_test_name);
    fprintf(stderr, "# Assertion failed: %s\n", expr);
    fprintf(stderr, "# Location: %s:%d\n", file, line);
    fflush(stderr);
    exit(EXIT_FAILURE);
}

#undef assert
#define assert(expr) test_assert_impl((expr) != 0, #expr, __FILE__, __LINE__)

static void
run_test(const char *name, test_fn fn, unsigned int *index)
{
    current_test_name = name;
    current_test_index = *index;
    printf("# %s\n", name);
    fn();
    printf("ok %u - %s\n", *index, name);
    *index += 1U;
}

static struct fpc_fir_config
fir_average_config(void)
{
    static const int32_t coeffs[] = {21845, 21845, 21845};
    struct fpc_fir_config cfg = {
        .order = 3U,
        .coeffs = coeffs
    };

    return cfg;
}

static struct fpc_biquad_config
biquad_identity_config(void)
{
    struct fpc_biquad_config cfg = {
        .b0 = 65536,
        .b1 = 0,
        .b2 = 0,
        .a1 = 0,
        .a2 = 0
    };

    return cfg;
}

static void
test_filter_pool_init_required(void)
{
    struct fpc_fir *fir = NULL;
    struct fpc_fir_config cfg = fir_average_config();

    assert(fpc_fir_init(&fir, &cfg) == FPC_STATUS_NOT_INITIALIZED);
    assert(fir == NULL);
}

static void
test_fir_init_and_process(void)
{
    struct fpc_fir *fir = NULL;
    struct fpc_fir_config cfg = fir_average_config();
    int32_t output = 0;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    assert(fpc_fir_init(&fir, &cfg) == FPC_STATUS_OK);
    assert(fpc_fir_process(fir, 1000, &output) == FPC_STATUS_OK);
    assert((output >= 330) && (output <= 340));
    assert(fpc_fir_process(fir, 2000, &output) == FPC_STATUS_OK);
    assert((output >= 990) && (output <= 1010));
    assert(fpc_fir_process(fir, 2000, &output) == FPC_STATUS_OK);
    assert((output >= 1650) && (output <= 1680));
    assert(fpc_fir_deinit(fir) == FPC_STATUS_OK);
}

static void
test_fir_reset_and_reconfigure(void)
{
    static const int32_t pass_coeffs[] = {65536};
    struct fpc_fir *fir = NULL;
    struct fpc_fir_config cfg = fir_average_config();
    struct fpc_fir_config pass_cfg = {
        .order = 1U,
        .coeffs = pass_coeffs
    };
    int32_t output = 0;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    assert(fpc_fir_init(&fir, &cfg) == FPC_STATUS_OK);
    assert(fpc_fir_process(fir, 900, &output) == FPC_STATUS_OK);
    assert(fpc_fir_reset(fir) == FPC_STATUS_OK);
    assert(fpc_fir_process(fir, 900, &output) == FPC_STATUS_OK);
    assert((output >= 290) && (output <= 310));
    assert(fpc_fir_set_config(fir, &pass_cfg) == FPC_STATUS_OK);
    assert(fpc_fir_process(fir, 1234, &output) == FPC_STATUS_OK);
    assert(output == 1234);
    assert(fpc_fir_deinit(fir) == FPC_STATUS_OK);
}

static void
test_fir_invalid_usage(void)
{
    struct fpc_fir *fir = NULL;
    int32_t coeffs[FPC_FILTER_MAX_ORDER + 1] = {0};
    struct fpc_fir_config invalid_cfg = {
        .order = (uint16_t)(FPC_FILTER_MAX_ORDER + 1U),
        .coeffs = coeffs
    };
    int32_t output = 0;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    assert(fpc_fir_init(NULL, &invalid_cfg) == FPC_STATUS_NULL_PTR);
    assert(fpc_fir_init(&fir, NULL) == FPC_STATUS_INVALID_PARAM);
    assert(fpc_fir_init(&fir, &invalid_cfg) == FPC_STATUS_INVALID_PARAM);
    assert(fpc_fir_process(NULL, 0, &output) == FPC_STATUS_NULL_PTR);
    assert(fpc_fir_process(fir, 0, NULL) == FPC_STATUS_NULL_PTR);
}

static void
test_fir_pool_exhaustion(void)
{
    struct fpc_fir *filters[FPC_MAX_INSTANCES] = {0};
    struct fpc_fir *extra = NULL;
    struct fpc_fir_config cfg = fir_average_config();
    uint16_t i;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    for (i = 0U; i < FPC_MAX_INSTANCES; ++i) {
        assert(fpc_fir_init(&filters[i], &cfg) == FPC_STATUS_OK);
    }

    assert(fpc_fir_init(&extra, &cfg) == FPC_STATUS_POOL_FULL);
    assert(extra == NULL);

    for (i = 0U; i < FPC_MAX_INSTANCES; ++i) {
        assert(fpc_fir_deinit(filters[i]) == FPC_STATUS_OK);
    }
}

static void
test_biquad_identity_and_reset(void)
{
    struct fpc_biquad *biquad = NULL;
    struct fpc_biquad_config cfg = biquad_identity_config();
    int32_t output = 0;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    assert(fpc_biquad_init(&biquad, &cfg) == FPC_STATUS_OK);
    assert(fpc_biquad_process(biquad, 1000, &output) == FPC_STATUS_OK);
    assert((output >= 900) && (output <= 1100));
    assert(fpc_biquad_reset(biquad) == FPC_STATUS_OK);
    assert(fpc_biquad_process(biquad, 2000, &output) == FPC_STATUS_OK);
    assert((output >= 1900) && (output <= 2100));
    assert(fpc_biquad_deinit(biquad) == FPC_STATUS_OK);
}

static void
test_biquad_reconfigure(void)
{
    struct fpc_biquad *biquad = NULL;
    struct fpc_biquad_config cfg = biquad_identity_config();
    struct fpc_biquad_config lowpass_cfg = {
        .b0 = 32768,
        .b1 = 32768,
        .b2 = 0,
        .a1 = 32768,
        .a2 = 0
    };
    int32_t output = 0;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    assert(fpc_biquad_init(&biquad, &cfg) == FPC_STATUS_OK);
    assert(fpc_biquad_set_config(biquad, &lowpass_cfg) == FPC_STATUS_OK);
    assert(fpc_biquad_process(biquad, 1000, &output) == FPC_STATUS_OK);
    assert((output >= 450) && (output <= 550));
    assert(fpc_biquad_process(biquad, 1000, &output) == FPC_STATUS_OK);
    assert((output >= 700) && (output <= 800));
    assert(fpc_biquad_deinit(biquad) == FPC_STATUS_OK);
}

static void
test_biquad_invalid_usage(void)
{
    struct fpc_biquad *biquad = NULL;
    struct fpc_biquad_config cfg = biquad_identity_config();
    int32_t output = 0;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    assert(fpc_biquad_init(NULL, &cfg) == FPC_STATUS_NULL_PTR);
    assert(fpc_biquad_init(&biquad, NULL) == FPC_STATUS_INVALID_PARAM);
    assert(fpc_biquad_process(NULL, 0, &output) == FPC_STATUS_NULL_PTR);
    assert(fpc_biquad_process(biquad, 0, NULL) == FPC_STATUS_NULL_PTR);
}

static void
test_filter_overflow_reporting(void)
{
    static const int32_t fir_coeffs[] = {INT32_MAX};
    struct fpc_fir *fir = NULL;
    struct fpc_fir_config fir_cfg = {
        .order = 1U,
        .coeffs = fir_coeffs
    };
    struct fpc_biquad *biquad = NULL;
    struct fpc_biquad_config biquad_cfg = {
        .b0 = INT32_MAX,
        .b1 = 0,
        .b2 = 0,
        .a1 = 0,
        .a2 = 0
    };
    int32_t output = 0;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    assert(fpc_fir_init(&fir, &fir_cfg) == FPC_STATUS_OK);
    assert(fpc_fir_process(fir, INT32_MAX, &output) == FPC_STATUS_OVERFLOW);
    assert(output == INT32_MAX);
    assert(fpc_fir_deinit(fir) == FPC_STATUS_OK);

    assert(fpc_biquad_init(&biquad, &biquad_cfg) == FPC_STATUS_OK);
    assert(fpc_biquad_process(biquad, INT32_MAX, &output) == FPC_STATUS_OVERFLOW);
    assert(output == INT32_MAX);
    assert(fpc_biquad_deinit(biquad) == FPC_STATUS_OK);
}

static void
test_biquad_pool_exhaustion(void)
{
    struct fpc_biquad *filters[FPC_MAX_INSTANCES] = {0};
    struct fpc_biquad *extra = NULL;
    struct fpc_biquad_config cfg = biquad_identity_config();
    uint16_t i;

    assert(fpc_filter_pool_init() == FPC_STATUS_OK);
    for (i = 0U; i < FPC_MAX_INSTANCES; ++i) {
        assert(fpc_biquad_init(&filters[i], &cfg) == FPC_STATUS_OK);
    }

    assert(fpc_biquad_init(&extra, &cfg) == FPC_STATUS_POOL_FULL);
    assert(extra == NULL);

    for (i = 0U; i < FPC_MAX_INSTANCES; ++i) {
        assert(fpc_biquad_deinit(filters[i]) == FPC_STATUS_OK);
    }
}

int
main(void)
{
    unsigned int test_index = 1U;

    setvbuf(stdout, NULL, _IONBF, 0);
    puts("TAP version 13");
    puts("1..10");

    run_test("filter_pool_init_required", test_filter_pool_init_required, &test_index);
    run_test("fir_init_and_process", test_fir_init_and_process, &test_index);
    run_test("fir_reset_and_reconfigure", test_fir_reset_and_reconfigure, &test_index);
    run_test("fir_invalid_usage", test_fir_invalid_usage, &test_index);
    run_test("fir_pool_exhaustion", test_fir_pool_exhaustion, &test_index);
    run_test("biquad_identity_and_reset", test_biquad_identity_and_reset, &test_index);
    run_test("biquad_reconfigure", test_biquad_reconfigure, &test_index);
    run_test("biquad_invalid_usage", test_biquad_invalid_usage, &test_index);
    run_test("filter_overflow_reporting", test_filter_overflow_reporting, &test_index);
    run_test("biquad_pool_exhaustion", test_biquad_pool_exhaustion, &test_index);

    return 0;
}
