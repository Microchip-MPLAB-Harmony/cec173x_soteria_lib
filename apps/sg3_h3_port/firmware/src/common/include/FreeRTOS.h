/*****************************************************************************
 * Copyright (c) 2022 Microchip Technology Inc. and its subsidiaries.
 * You may use this software and any derivatives exclusively with
 * Microchip products.
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".
 * NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP
 * PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.
 * TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
 * CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF
 * FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE
 * OF THESE TERMS.
 *****************************************************************************/

#ifndef INCLUDE_FREERTOS_H_
#define INCLUDE_FREERTOS_H_

typedef uint32_t     TickType_t;
typedef unsigned long    UBaseType_t;
typedef unsigned long    UBaseType_t;
typedef long             BaseType_t;
#define config_CEC_AHB_PROTECTION_ENABLE 1
#define portUSING_MPU_WRAPPERS 1
#define configSUPPORT_STATIC_ALLOCATION 1
typedef void (* TaskFunction_t)( void * );
#define configSTACK_DEPTH_TYPE    uint16_t
typedef unsigned long    UBaseType_t;
#define portSTACK_TYPE    uint32_t
typedef portSTACK_TYPE   StackType_t;
/* Max number of MCTP applications */
#define DI_MAX_MCTP_APPS		2
// SPDM configs
#define SPDM_MAX_QMSPI_PORTS	2		// PLDM uses both ports depending on the APx
#define SPDM_CRYPTO_BLOCKS_ACCESS	2	// SPDM needs access to both Block1 and Block2 crypto assets

#define portTOTAL_NUM_REGIONS 8

#define configTICK_RATE_HZ (200)
#define pdMS_TO_TICKS( xTimeInMs )    ( ( TickType_t ) ( ( ( TickType_t ) ( xTimeInMs ) * ( TickType_t ) configTICK_RATE_HZ ) / ( TickType_t ) 1000U ) )

typedef struct _CEC_AHB_PRIV_REGS_VALUES_
{
    /* Chip Privilege enable values*/
    uint32_t chip_priv_en;
    
    /* EC Privilege enable values */
    uint32_t ec_priv_en;
    
    /* EC Privilege enable 2 values */
    uint32_t ec_priv_en2;
    
    /* EC Privilege enable 3 values */
    uint32_t ec_priv_en3;    
    
}CEC_AHB_PRIV_REGS_VALUES;

typedef struct MPU_REGION_REGISTERS
{
    uint32_t ulRegionBaseAddress;
    uint32_t ulRegionAttribute;
} xMPU_REGION_REGISTERS;

/* Plus 1 to create space for the stack region. */
typedef struct MPU_SETTINGS
{
    xMPU_REGION_REGISTERS xRegion[ portTOTAL_NUM_REGIONS ];
#ifdef config_CEC_AHB_PROTECTION_ENABLE
    CEC_AHB_PRIV_REGS_VALUES  cecAHBprivRegValues;
#endif    
} xMPU_SETTINGS;
#define configMAX_TASK_NAME_LEN    16
struct xSTATIC_LIST_ITEM
{
    #if ( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 1 )
        TickType_t xDummy1;
    #endif
    TickType_t xDummy2;
    void * pvDummy3[ 4 ];
    #if ( configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES == 1 )
        TickType_t xDummy4;
    #endif
};
typedef struct xSTATIC_LIST_ITEM StaticListItem_t;

typedef struct xSTATIC_TCB
{
    void * pxDummy1;
    #if ( portUSING_MPU_WRAPPERS == 1 )
        xMPU_SETTINGS xDummy2;
    #endif
    StaticListItem_t xDummy3[ 2 ];
    UBaseType_t uxDummy5;
    void * pxDummy6;
    uint8_t ucDummy7[ configMAX_TASK_NAME_LEN ];
    #if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
        void * pxDummy8;
    #endif
    #if ( portCRITICAL_NESTING_IN_TCB == 1 )
        UBaseType_t uxDummy9;
    #endif
    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t uxDummy10[ 2 ];
    #endif
    #if ( configUSE_MUTEXES == 1 )
        UBaseType_t uxDummy12[ 2 ];
    #endif
    #if ( configUSE_APPLICATION_TASK_TAG == 1 )
        void * pxDummy14;
    #endif
    #if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
        void * pvDummy15[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
    #endif
    #if ( configGENERATE_RUN_TIME_STATS == 1 )
        uint32_t ulDummy16;
    #endif
    #if ( configUSE_NEWLIB_REENTRANT == 1 )
        struct  _reent xDummy17;
    #endif
    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        uint32_t ulDummy18[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
        uint8_t ucDummy19[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
    #endif
    #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 )
        uint8_t uxDummy20;
    #endif

    #if ( INCLUDE_xTaskAbortDelay == 1 )
        uint8_t ucDummy21;
    #endif
    #if ( configUSE_POSIX_ERRNO == 1 )
        int iDummy22;
    #endif
} StaticTask_t;

typedef struct xMEMORY_REGION
{
    void * pvBaseAddress;
    uint32_t ulLengthInBytes;
    uint32_t ulParameters;
} MemoryRegion_t;

#define portNUM_CONFIGURABLE_REGIONS    3

typedef struct xTASK_PARAMETERS
{
    TaskFunction_t pvTaskCode;
    const char * const pcName;     /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
    configSTACK_DEPTH_TYPE usStackDepth;
    void * pvParameters;
    UBaseType_t uxPriority;
    StackType_t * puxStackBuffer;
#ifdef config_CEC_AHB_PROTECTION_ENABLE
    CEC_AHB_PRIV_REGS_VALUES *pCecPrivRegValues;
#endif    
    MemoryRegion_t xRegions[ portNUM_CONFIGURABLE_REGIONS ];
    #if ( ( portUSING_MPU_WRAPPERS == 1 ) && ( configSUPPORT_STATIC_ALLOCATION == 1 ) )
        StaticTask_t * const pxTaskBuffer;
    #endif
} TaskParameters_t;

extern void vAssertCalled(unsigned long ulLine, const char *const pcFileName);
#define configASSERT(x)                                                \
    if ((unsigned long)(x) == 0UL) {                                   \
        vAssertCalled((unsigned long )__LINE__,                        \
                      (const char *)__FILE__);                         \
    }

#define pdFALSE                                  ( ( BaseType_t ) 0 )
#define pdTRUE                                   ( ( BaseType_t ) 1 )

#define pdPASS                                   ( pdTRUE )
#define pdFAIL                                   ( pdFALSE )

#define listFIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE
#define listSECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE
#define configLIST_VOLATILE

struct xLIST_ITEM
{
    listFIRST_LIST_ITEM_INTEGRITY_CHECK_VALUE               /*< Set to a known value if configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES is set to 1. */
    configLIST_VOLATILE TickType_t xItemValue;              /*< The value being listed.  In most cases this is used to sort the list in descending order. */
    struct xLIST_ITEM * configLIST_VOLATILE pxNext;         /*< Pointer to the next ListItem_t in the list. */
    struct xLIST_ITEM * configLIST_VOLATILE pxPrevious;     /*< Pointer to the previous ListItem_t in the list. */
    void * pvOwner;                                         /*< Pointer to the object (normally a TCB) that contains the list item.  There is therefore a two way link between the object containing the list item and the list item itself. */
    struct xLIST * configLIST_VOLATILE pxContainer;         /*< Pointer to the list in which this list item is placed (if any). */
    listSECOND_LIST_ITEM_INTEGRITY_CHECK_VALUE              /*< Set to a known value if configUSE_LIST_DATA_INTEGRITY_CHECK_BYTES is set to 1. */
};
typedef struct xLIST_ITEM ListItem_t; 
typedef struct tskTaskControlBlock       /* The old naming convention is used to prevent breaking kernel aware debuggers. */
{
    volatile StackType_t * pxTopOfStack; /*< Points to the location of the last item placed on the tasks stack.  THIS MUST BE THE FIRST MEMBER OF THE TCB STRUCT. */

    #if ( portUSING_MPU_WRAPPERS == 1 )
        xMPU_SETTINGS xMPUSettings; /*< The MPU settings are defined as part of the port layer.  THIS MUST BE THE SECOND MEMBER OF THE TCB STRUCT. */
    #endif

    ListItem_t xStateListItem;                  /*< The list that the state list item of a task is reference from denotes the state of that task (Ready, Blocked, Suspended ). */
    ListItem_t xEventListItem;                  /*< Used to reference a task from an event list. */
    UBaseType_t uxPriority;                     /*< The priority of the task.  0 is the lowest priority. */
    StackType_t * pxStack;                      /*< Points to the start of the stack. */
    char pcTaskName[ configMAX_TASK_NAME_LEN ]; /*< Descriptive name given to the task when created.  Facilitates debugging only. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */

    #if ( ( portSTACK_GROWTH > 0 ) || ( configRECORD_STACK_HIGH_ADDRESS == 1 ) )
        StackType_t * pxEndOfStack; /*< Points to the highest valid address for the stack. */
    #endif

    #if ( portCRITICAL_NESTING_IN_TCB == 1 )
        UBaseType_t uxCriticalNesting; /*< Holds the critical section nesting depth for ports that do not maintain their own count in the port layer. */
    #endif

    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t uxTCBNumber;  /*< Stores a number that increments each time a TCB is created.  It allows debuggers to determine when a task has been deleted and then recreated. */
        UBaseType_t uxTaskNumber; /*< Stores a number specifically for use by third party trace code. */
    #endif

    #if ( configUSE_MUTEXES == 1 )
        UBaseType_t uxBasePriority; /*< The priority last assigned to the task - used by the priority inheritance mechanism. */
        UBaseType_t uxMutexesHeld;
    #endif

    #if ( configUSE_APPLICATION_TASK_TAG == 1 )
        TaskHookFunction_t pxTaskTag;
    #endif

    #if ( configNUM_THREAD_LOCAL_STORAGE_POINTERS > 0 )
        void * pvThreadLocalStoragePointers[ configNUM_THREAD_LOCAL_STORAGE_POINTERS ];
    #endif

    #if ( configGENERATE_RUN_TIME_STATS == 1 )
        uint32_t ulRunTimeCounter; /*< Stores the amount of time the task has spent in the Running state. */
    #endif

    #if ( configUSE_NEWLIB_REENTRANT == 1 )

        /* Allocate a Newlib reent structure that is specific to this task.
         * Note Newlib support has been included by popular demand, but is not
         * used by the FreeRTOS maintainers themselves.  FreeRTOS is not
         * responsible for resulting newlib operation.  User must be familiar with
         * newlib and must provide system-wide implementations of the necessary
         * stubs. Be warned that (at the time of writing) the current newlib design
         * implements a system-wide malloc() that must be provided with locks.
         *
         * See the third party link http://www.nadler.com/embedded/newlibAndFreeRTOS.html
         * for additional information. */
        struct  _reent xNewLib_reent;
    #endif

    #if ( configUSE_TASK_NOTIFICATIONS == 1 )
        volatile uint32_t ulNotifiedValue[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
        volatile uint8_t ucNotifyState[ configTASK_NOTIFICATION_ARRAY_ENTRIES ];
    #endif

    /* See the comments in FreeRTOS.h with the definition of
     * tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE. */
    #if ( tskSTATIC_AND_DYNAMIC_ALLOCATION_POSSIBLE != 0 ) /*lint !e731 !e9029 Macro has been consolidated for readability reasons. */
        uint8_t ucStaticallyAllocated;                     /*< Set to pdTRUE if the task is a statically allocated to ensure no attempt is made to free the memory. */
    #endif

    #if ( INCLUDE_xTaskAbortDelay == 1 )
        uint8_t ucDelayAborted;
    #endif

    #if ( configUSE_POSIX_ERRNO == 1 )
        int iTaskErrno;
    #endif
} tskTCB;

struct tmrTimerControl; /* The old naming convention is used to prevent breaking kernel aware debuggers. */
typedef struct tmrTimerControl * TimerHandle_t;
typedef void (* TimerCallbackFunction_t)( TimerHandle_t xTimer );

/* The definition of the timers themselves. */
typedef struct tmrTimerControl                  /* The old naming convention is used to prevent breaking kernel aware debuggers. */
{
    const char * pcTimerName;                   /*<< Text name.  This is not used by the kernel, it is included simply to make debugging easier. */ /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
    ListItem_t xTimerListItem;                  /*<< Standard linked list item as used by all kernel features for event management. */
    TickType_t xTimerPeriodInTicks;             /*<< How quickly and often the timer expires. */
    void * pvTimerID;                           /*<< An ID to identify the timer.  This allows the timer to be identified when the same callback is used for multiple timers. */
    TimerCallbackFunction_t pxCallbackFunction; /*<< The function that will be called when the timer expires. */
    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t uxTimerNumber;              /*<< An ID assigned by trace tools such as FreeRTOS+Trace */
    #endif
    uint8_t ucStatus;                           /*<< Holds bits to say if the timer was statically allocated or not, and if it is active or not. */
} xTIMER;

typedef struct xSTATIC_TIMER
{
    void * pvDummy1;
    StaticListItem_t xDummy2;
    TickType_t xDummy3;
    void * pvDummy5;
    TaskFunction_t pvDummy6;
    #if ( configUSE_TRACE_FACILITY == 1 )
        UBaseType_t uxDummy7;
    #endif
    uint8_t ucDummy8;
} StaticTimer_t;

struct tskTaskControlBlock;     /* The old naming convention is used to prevent breaking kernel aware debuggers. */
typedef struct tskTaskControlBlock * TaskHandle_t;

void di_checks_register_task(TaskHandle_t app_task_handle, uint8_t app_id);
BaseType_t xTaskCreateRestrictedStatic( const TaskParameters_t * const pxTaskDefinition,
                                            TaskHandle_t * pxCreatedTask );

typedef uint32_t   * QueueHandle_t;
typedef QueueHandle_t SemaphoreHandle_t;
typedef TickType_t               EventBits_t;
#define STR_VALUE(arg)    #arg
#define FUNCTION_NAME(name)   STR_VALUE(name)
#define PRIV_FUNC(x)    privileged_functions_##x
#define PRIV_FUNC_NAME(x)   FUNCTION_NAME(PRIV_FUNC(x))
#define PRIVILEGED_FUNCTION     __attribute__( ( section( PRIV_FUNC_NAME(__LINE__) ) ) )
BaseType_t MPU_xQueueGenericSend( QueueHandle_t xQueue,
                              const void * const pvItemToQueue,
                              TickType_t xTicksToWait,
                              const BaseType_t xCopyPosition ) PRIVILEGED_FUNCTION;
#define queueSEND_TO_BACK                     ( ( BaseType_t ) 0 )
#define semGIVE_BLOCK_TIME                  ( ( TickType_t ) 0U )
#define xSemaphoreGive( xSemaphore )    MPU_xQueueGenericSend( ( QueueHandle_t ) ( xSemaphore ), NULL, semGIVE_BLOCK_TIME, queueSEND_TO_BACK )


 #define portMAX_DELAY              ( TickType_t ) 0xffffffffUL
typedef uint32_t * EventGroupHandle_t;
typedef long             BaseType_t;
EventBits_t MPU_xEventGroupSetBits( EventGroupHandle_t xEventGroup,
                                const EventBits_t uxBitsToSet );
#define xEventGroupSetBits MPU_xEventGroupSetBits
EventBits_t MPU_xEventGroupWaitBits( EventGroupHandle_t xEventGroup,
                                 const EventBits_t uxBitsToWaitFor,
                                 const BaseType_t xClearOnExit,
                                 const BaseType_t xWaitForAllBits,
                                 TickType_t xTicksToWait );
#define xEventGroupWaitBits MPU_xEventGroupWaitBits
TimerHandle_t MPU_xTimerCreateStatic( const char * const pcTimerName,
                                      const TickType_t xTimerPeriodInTicks,
                                      const UBaseType_t uxAutoReload,
                                      void * const pvTimerID,
                                      TimerCallbackFunction_t pxCallbackFunction,
                                      StaticTimer_t * pxTimerBuffer );
#define xTimerCreateStatic MPU_xTimerCreateStatic

#define xTimerGenericCommand MPU_xTimerGenericCommand
BaseType_t xTimerGenericCommand( TimerHandle_t xTimer,
                                 const BaseType_t xCommandID,
                                 const TickType_t xOptionalValue,
                                 BaseType_t * const pxHigherPriorityTaskWoken,
                                 const TickType_t xTicksToWait );
TickType_t xTaskGetTickCount( void ) PRIVILEGED_FUNCTION;
#define tmrCOMMAND_START                        ( ( BaseType_t ) 1 )
#define xTimerStart( xTimer, xTicksToWait ) \
    xTimerGenericCommand( ( xTimer ), tmrCOMMAND_START, ( xTaskGetTickCount() ), NULL, ( xTicksToWait ) )
#define tmrCOMMAND_STOP                         ( ( BaseType_t ) 3 )
#define xTimerStop( xTimer, xTicksToWait ) \
    xTimerGenericCommand( ( xTimer ), tmrCOMMAND_STOP, 0U, NULL, ( xTicksToWait ) )

typedef struct ST_DI_QUEUE_EVENT
{
    /** Pointer to message queue handle */
    QueueHandle_t queue_handle;

    /** Event Group handle to notify the task associated with the message queue */
    EventGroupHandle_t  evt_grp_handle;

    /** Pointer to event queue handle */
    QueueHandle_t event_queue_handle;

} DI_QUEUE_EVENT;

typedef struct ST_DI_CONTEXT_MCTP_MASTER
{
    /** SPDM Request Message Queue and event group handle */
    DI_QUEUE_EVENT request;

    /** Message Queue and event group handles of MCTP apps */
    DI_QUEUE_EVENT app_info[DI_MAX_MCTP_APPS];

    /** Semaphores for each application */
    SemaphoreHandle_t mutex_handles[DI_MAX_MCTP_APPS];

} DI_CONTEXT_MCTP_MASTER;

typedef struct ST_DI_CONTEXT_MCTP
{
    // SMB Request INFO
    /** SMB Request Message Queue and event group handle */
    DI_QUEUE_EVENT smb_request;

    /** Semaphore for MCTP smb channel */
    SemaphoreHandle_t mutex_smb_handle;

    /** Slave TX Message Queue handle */
    QueueHandle_t slave_tx_msg_queue_handle;

    /** MCTP Message Queue and event group handle */
    DI_QUEUE_EVENT mctp_request;

    /* Context when MCTP is acting as a master to slave applications */
    DI_CONTEXT_MCTP_MASTER mctp_master_context;

    /** SB_CORE Request Message Queue and event group handle */
    DI_QUEUE_EVENT sb_core_request;

    /** Semaphore for SPDM SB_CORE request queue */
    SemaphoreHandle_t mutex_sb_core_handle;

} DI_CONTEXT_MCTP;


/******************************************************************************/
/**  SPDM Data Isolation init members
*******************************************************************************/
typedef struct ST_DI_CONTEXT_SPDM
{
    // MCTP Request INFO
    /** MCTP Request Message Queue and event group handle */
    DI_QUEUE_EVENT mctp_request;

    /** Semaphore for SPDM MCTP channel */
    SemaphoreHandle_t mutex_mctp_handle;

    /** SPDM Message Queue and event group handle */
    DI_QUEUE_EVENT spdm_request;

    /** QMSPI Request Message Queue and event group handle */
    DI_QUEUE_EVENT qmspi_request;

    /** Semaphore for SPDM QMSPI channel */
    SemaphoreHandle_t mutex_qmspi_handle[SPDM_MAX_QMSPI_PORTS];

    /** SPDM response message queue and event group handle to for QMSPI transactions */
    DI_QUEUE_EVENT spdm_qmspi_response;

    /** SB_CORE Request Message Queue and event group handle */
    DI_QUEUE_EVENT sb_core_request;

    /** Semaphore for SPDM SB_CORE request queue */
    SemaphoreHandle_t mutex_sb_core_handle;

    /** SPDM response message queue and event group handle to for sb_core transactions */
    DI_QUEUE_EVENT spdm_sb_core_response;

    /** Crypto Request Message Queue and event group handle */
    DI_QUEUE_EVENT crypto_request;

    /** Semaphore for Crypto request queue */
    SemaphoreHandle_t mutex_crypto_handles[SPDM_CRYPTO_BLOCKS_ACCESS];

    /** SPDM response message queue and event group handle to for crypto transactions */
    DI_QUEUE_EVENT spdm_crypto_response;

    /** Semaphore for I2C request queue */
    SemaphoreHandle_t mutex_i2c_handle;

    /** I2C Request Message Queue and event group handle */
    DI_QUEUE_EVENT i2c_response; // get response from i2c
    DI_QUEUE_EVENT i2c_request; // Send request to i2c

} DI_CONTEXT_SPDM;

#endif
