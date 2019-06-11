#ifndef _WRAPPERS_DEFS_H_
#define _WRAPPERS_DEFS_H_

#include "infra_types.h"
#include "infra_defs.h"

#define PLATFORM_WAIT_INFINITE (~0)

typedef struct {
    void *(*malloc)(uint32_t size);
    void (*free)(void *ptr);
} ssl_hooks_t;

typedef enum {
    os_thread_priority_idle = -3,        /* priority: idle (lowest) */
    os_thread_priority_low = -2,         /* priority: low */
    os_thread_priority_belowNormal = -1, /* priority: below normal */
    os_thread_priority_normal = 0,       /* priority: normal (default) */
    os_thread_priority_aboveNormal = 1,  /* priority: above normal */
    os_thread_priority_high = 2,         /* priority: high */
    os_thread_priority_realtime = 3,     /* priority: realtime (highest) */
    os_thread_priority_error = 0x84,     /* system cannot determine priority or thread has illegal priority */
} hal_os_thread_priority_t;

typedef struct _hal_os_thread {
    hal_os_thread_priority_t priority;     /*initial thread priority */
    void                    *stack_addr;   /* thread stack address malloced by caller, use system stack by . */
    int                   stack_size;   /* stack size requirements in bytes; 0 is default stack size */
    int                      detach_state; /* 0: not detached state; otherwise: detached state. */
    char                    *name;         /* thread name. */
} hal_os_thread_param_t;

#define DTLS_ERROR_BASE                (1<<24)
#define DTLS_SUCCESS                   (0)
#define DTLS_INVALID_PARAM             (DTLS_ERROR_BASE | 1)
#define DTLS_INVALID_CA_CERTIFICATE    (DTLS_ERROR_BASE | 2)
#define DTLS_HANDSHAKE_IN_PROGRESS     (DTLS_ERROR_BASE | 3)
#define DTLS_HANDSHAKE_FAILED          (DTLS_ERROR_BASE | 4)
#define DTLS_FATAL_ALERT_MESSAGE       (DTLS_ERROR_BASE | 5)
#define DTLS_PEER_CLOSE_NOTIFY         (DTLS_ERROR_BASE | 6)
#define DTLS_SESSION_CREATE_FAILED     (DTLS_ERROR_BASE | 7)
#define DTLS_READ_DATA_FAILED          (DTLS_ERROR_BASE | 8)

typedef struct {
    void *(*malloc)(uint32_t size);
    void (*free)(void *ptr);
} dtls_hooks_t;

typedef struct {
    unsigned char             *p_ca_cert_pem;
    char                      *p_host;
    unsigned short             port;
} coap_dtls_options_t;

typedef void DTLSContext;

typedef void (*timer_callback)(void *);

extern void HAL_Free(void *ptr);
extern int HAL_Kv_Del(const char *key);
extern int HAL_Kv_Get(const char *key, void *buffer, int *buffer_len);
extern int HAL_Kv_Set(const char *key, const void *val, int len, int sync);
extern void *HAL_Malloc(uint32_t size);
extern void *HAL_MutexCreate(void);
extern void HAL_MutexDestroy(void *mutex);
extern void HAL_MutexLock(void *mutex);
extern void HAL_MutexUnlock(void *mutex);
extern uint32_t HAL_Random(uint32_t region);
extern void *HAL_SemaphoreCreate(void);
extern void HAL_SemaphoreDestroy(_IN_ void *sem);
extern void HAL_SemaphorePost(_IN_ void *sem);
extern int HAL_SemaphoreWait(_IN_ void *sem, _IN_ uint32_t timeout_ms);
extern int HAL_SetDeviceSecret(char *device_secret);
extern void HAL_SleepMs(uint32_t ms);
extern void HAL_Srandom(uint32_t seed);
extern int HAL_ThreadCreate(
                _OU_ void **thread_handle,
                _IN_ void *(*work_routine)(void *),
                _IN_ void *arg,
                _IN_ hal_os_thread_param_t *hal_os_thread_param,
                _OU_ int *stack_used);
extern void HAL_ThreadDelete(_IN_ void *thread_handle);
extern void *HAL_Timer_Create(const char *name, timer_callback func, void *user_data);
extern int HAL_Timer_Delete(void *timer);
extern int HAL_Timer_Start(void *timer, int ms);
extern int HAL_Timer_Stop(void *timer);
extern void *HAL_Timer_Task(void *para);
extern int HAL_Timer_Task_Init();
extern uint64_t HAL_UptimeMs(void);

#endif

