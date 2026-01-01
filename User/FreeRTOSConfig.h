/*
 * FreeRTOSConfig.h
 * 适用于 STM32F103C8 (Blue Pill) + Keil (不使用 HAL，寄存器/StdPeriph 风格)
 *
 * 说明：
 * - 假定 CPU 时钟 72MHz（如不一样请修改 configCPU_CLOCK_HZ）
 * - Tick 1000Hz
 * - 使用 heap_4.c（可换为 heap_1/2/3/5）
 * - 将此文件放到工程 include 路径中并与 FreeRTOS 源一起编译
 *
 * 根据你的工程需要微调 configTOTAL_HEAP_SIZE、configMAX_PRIORITIES、栈大小等。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*-----------------------------------------------------------
 * Kernel / scheduler configuration
 *----------------------------------------------------------*/
#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      ( ( uint32_t ) 72000000 )   /* 根据实际 SystemCoreClock 修改 */
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 5 )
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 128 )        /* 以字 (words) 为单位 */
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 13 * 1024 ) )/* Blue Pill RAM 有限，按需调整 */
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/*-----------------------------------------------------------
 * API function inclusion
 *----------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_vTaskCleanUpResources           0

/*-----------------------------------------------------------
 * Optional features
 *----------------------------------------------------------*/
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 2 )
#define configTIMER_QUEUE_LENGTH                5
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE )

#define configUSE_MALLOC_FAILED_HOOK            0
#define configCHECK_FOR_STACK_OVERFLOW          2   /* 0=禁用，1=简单检测，2=增强检测 */

/* Run time and task stats (禁用以节省空间，按需开启) */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/*-----------------------------------------------------------
 * Cortex-M specific definitions
 *----------------------------------------------------------*/
#ifdef __NVIC_PRIO_BITS
    /* CMSIS 中定义的优先位数 */
    #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
    /* STM32F1 通常为 4 */
    #define configPRIO_BITS         4
#endif

/* 可用的最低中断优先级（数值越大优先级越低）*/
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15

/* 从中断中调用 FreeRTOS API 的最高可用优先级（数值越小优先级越高）
   — 这是一个保守值，按需要调整（保证此值 >= 0 且 < (1<<configPRIO_BITS)）*/
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

/* 转换为 NVIC 使用的优先级值 */
#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* 中断安全性的断言 */
#define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }

/* 优化任务选择（Cortex-M 支持位操作选择）*/
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1

/*-----------------------------------------------------------
 * Names for the FreeRTOS port interrupt handlers.
 * If you use the standard startup vector table (which calls SVC_Handler,
 * PendSV_Handler and SysTick_Handler), map the port handlers to those names.
 *----------------------------------------------------------*/
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/*-----------------------------------------------------------
 * Optional: application specific defines
 *----------------------------------------------------------*/
#define configAPPLICATION_ALLOCATED_HEAP         0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0

/* 仅使用动态分配，关闭静态分配以缩减占用 */
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CONFIG_H */
