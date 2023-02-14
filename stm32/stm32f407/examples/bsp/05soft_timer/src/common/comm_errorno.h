/**
 * @file comm_errorno.h
 * @brief Generic error definition header file
 * @copyright Copyright (c) 2023
 *
 * Change Logs:
 * Data             Author                          Notes
 * 2023-02-09       vector(vector_qiu@163.com)      first version
 *
 */
#ifndef __COMM_ERRORNO_H__
#define __COMM_ERRORNO_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t error_t;

#define EOK                          0               /**< There is no error */
#define ERROR                        1               /**< A generic error happens */
#define ETIMEOUT                     2               /**< Timed out */
#define EFULL                        3               /**< The resource is full */
#define EEMPTY                       4               /**< The resource is empty */
#define ENOMEM                       5               /**< No memory */
#define ENOSYS                       6               /**< No system */
#define EBUSY                        7               /**< Busy */
#define EIO                          8               /**< IO error */
#define EINTR                        9               /**< Interrupted system call */
#define EINVAL                       10              /**< Invalid argument */

#ifdef __cplusplus
}
#endif

#endif /* __COMM_ERRORNO_H__ */
