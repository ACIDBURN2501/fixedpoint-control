/**
 * @file fpc_pool_config.h
 * @brief Pool allocator configuration bridge for fixedpoint-control.
 *
 * This header adapts the generated fixedpoint-control build configuration to
 * the bundled pool allocator's compile-time macro interface.
 */

#ifndef FPC_POOL_CONFIG_H_
#define FPC_POOL_CONFIG_H_

#include "fpc_build_config.h"

#ifndef POOL_ITEM_SIZE
#define POOL_ITEM_SIZE FPC_POOL_ITEM_SIZE
#endif

#ifndef POOL_MAX_SLOTS
#define POOL_MAX_SLOTS FPC_MAX_INSTANCES
#endif

#endif /* FPC_POOL_CONFIG_H_ */
