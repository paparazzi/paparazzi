#pragma once
#include "stdbool.h"
#include "threads_arch.h"
#include "stdint.h"


typedef struct pprzMutex pprz_mutex_t;
typedef struct pprzBSem pprz_bsem_t;


int pprz_mtx_init(pprz_mutex_t* mtx);
int pprz_mtx_lock(pprz_mutex_t* mtx);

/**
 * @brief Performs a nonblocking lock on the mutex.
 * @return 0 if successful
 */
int pprz_mtx_trylock(pprz_mutex_t* mtx);
int pprz_mtx_unlock(pprz_mutex_t* mtx);

void pprz_bsem_init(pprz_bsem_t* bsem, bool taken);
void pprz_bsem_wait(pprz_bsem_t* bsem);
void pprz_bsem_signal(pprz_bsem_t* bsem);

/**
 * @brief   Creates a new thread whose stack is dynamically allocated.
 * @param[in] size      size of the stack. Unused on linux at the moment.
 * @param[in] prio      the priority level for the new thread. Unused on linux at the moment. NORMALPRIO
 * @param[in] pf        the thread function
 * @param[in] arg       an argument passed to the thread function. It can be
 *                      @p NULL.
 * @param[out] thread    pointer to a pprz_thread_t that will be writen with the handle to the thread
 * @return              Return 0 on success, or a platform dependant error code.
 *
 * @api
 */
int pprz_thread_create(pprz_thread_t* thread, size_t size, const char *name, uint8_t prio, void (*func)(void*), void* arg);

/**
 * @brief   Exit the current thread
 * @param[out] retval   The thread's return value
 */
void pprz_thread_exit(void *retval);

/**
 * @brief Wait for the thread to terminate.
 * @note  Warning ! This is a blocking function, do not use it in the main AP thread !
 * @param[in] thread    The thread to be joined
 * @param[out] retval   The thread's return value will be written to the variable pointed by retval
 */
int pprz_thread_join(pprz_thread_t* thread, void** retval);


/**
 * @brief Performs a nonblocking join with the thread.
 * @param[in] thread    The thread to be joined
 * @param[out] retval   The thread's return value will be written to the variable pointed by retval.
 *                      must not be used if this fuction did not returned 0.
 * @return        0 if the thread successfully joined.
 */
int pprz_thread_tryjoin(pprz_thread_t* thread, void** retval);
