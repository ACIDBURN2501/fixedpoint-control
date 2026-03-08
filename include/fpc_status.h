/**
 * @file fpc_status.h
 * @brief Shared status codes for fixedpoint-control APIs.
 */

#ifndef FPC_STATUS_H_
#define FPC_STATUS_H_

/**
 * @enum fpc_status
 * @brief Common return codes for public APIs.
 *
 * Negative values indicate errors. Positive values indicate successful
 * operations with a noteworthy condition, such as saturation.
 */
enum fpc_status {
    FPC_STATUS_OK = 0,
    FPC_STATUS_SATURATED = 1,
    FPC_STATUS_NULL_PTR = -1,
    FPC_STATUS_INVALID_PARAM = -2,
    FPC_STATUS_NOT_INITIALIZED = -3,
    FPC_STATUS_POOL_FULL = -4,
    FPC_STATUS_OVERFLOW = -5
};

#endif /* FPC_STATUS_H_ */
