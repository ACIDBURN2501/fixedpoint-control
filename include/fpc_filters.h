/**
 * @file fpc_filters.h
 * @brief Safety-oriented FIR and biquad filter APIs.
 */

#ifndef FPC_FILTERS_H_
#define FPC_FILTERS_H_

#include <stdint.h>
#include "fpc_status.h"

#ifndef FPC_FILTER_MAX_ORDER
#define FPC_FILTER_MAX_ORDER 64U
#endif

struct fpc_fir;
struct fpc_biquad;

/**
 * @struct fpc_fir_config
 * @brief FIR filter configuration.
 */
struct fpc_fir_config {
    uint16_t order;
    const int32_t *coeffs;
};

/**
 * @struct fpc_biquad_config
 * @brief Biquad coefficient set in Q16.16.
 */
struct fpc_biquad_config {
    int32_t b0;
    int32_t b1;
    int32_t b2;
    int32_t a1;
    int32_t a2;
};

enum fpc_status fpc_filter_pool_init(void);
enum fpc_status fpc_fir_init(struct fpc_fir **ctx,
                             const struct fpc_fir_config *cfg);
enum fpc_status fpc_fir_reset(struct fpc_fir *ctx);
enum fpc_status fpc_fir_set_config(struct fpc_fir *ctx,
                                   const struct fpc_fir_config *cfg);
enum fpc_status fpc_fir_process(struct fpc_fir *ctx,
                                int32_t sample,
                                int32_t *output);
enum fpc_status fpc_fir_deinit(struct fpc_fir *ctx);

enum fpc_status fpc_biquad_init(struct fpc_biquad **ctx,
                                const struct fpc_biquad_config *cfg);
enum fpc_status fpc_biquad_reset(struct fpc_biquad *ctx);
enum fpc_status fpc_biquad_set_config(struct fpc_biquad *ctx,
                                      const struct fpc_biquad_config *cfg);
enum fpc_status fpc_biquad_process(struct fpc_biquad *ctx,
                                   int32_t sample,
                                   int32_t *output);
enum fpc_status fpc_biquad_deinit(struct fpc_biquad *ctx);

#endif /* FPC_FILTERS_H_ */
