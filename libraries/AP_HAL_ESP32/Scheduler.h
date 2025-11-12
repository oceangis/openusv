/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ESP32_SCHEDULER_MAX_TIMER_PROCS 10
#define ESP32_SCHEDULER_MAX_IO_PROCS 10
#define TWDT_TIMEOUT_MS 3000

/* Scheduler implementation: */
class ESP32::Scheduler : public AP_HAL::Scheduler
{

public:
    Scheduler();
    ~Scheduler();
    /* AP_HAL::Scheduler methods */
    void     init() override;
    void     set_callbacks(AP_HAL::HAL::Callbacks *cb)
    {
        callbacks = cb;
    };
    void     delay(uint16_t ms) override;
    void     delay_microseconds(uint16_t us) override;
    void     delay_microseconds_boost(uint16_t us) override;
    void     boost_end(void) override;
    void     register_timer_process(AP_HAL::MemberProc) override;
    void     register_io_process(AP_HAL::MemberProc) override;
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us) override;
    void     reboot(bool hold_in_bootloader) override;
    bool     in_main_thread() const override;
    // check and set the startup state
    void     set_system_initialized() override;
    bool     is_system_initialized() override;

    // Expected delay management (suppress watchdog warnings)
    void     expect_delay_ms(uint32_t ms) override;
    bool     in_expected_delay(void) const override;

    void     print_stats(void) ;
    void     print_main_loop_rate(void);

    uint16_t get_loop_rate_hz(void);
    AP_Int16 _active_loop_rate_hz;
    AP_Int16 _loop_rate_hz;

    static void thread_create_trampoline(void *ctx);
    bool thread_create(AP_HAL::MemberProc, const char *name, uint32_t stack_size, priority_base base, int8_t priority) override;

    // configMAX_PRIORITIES=25
    // Priority design based on ChibiOS with tighter grouping to prevent task starvation

    // High Priority Group (20-24): Critical real-time tasks
    static const int MAIN_PRIO           = 22;  // Main loop
    static const int MAIN_PRIO_BOOST     = 23;  // Main loop boosted (only +1 from normal)
    static const int TIMER_PRIO          = 23;  // Timer thread (critical timing)
    static const int UART_PRIO           = 22;  // UART high priority for telemetry
    static const int SPI_PRIORITY        = 23;  // SPI for IMU (critical sensor data)
    static const int RCOUT_PRIO          = 23;  // PWM output (critical for control)

    // Medium Priority Group (15-19): Important but not critical
    static const int WIFI_PRIO1          = 18;  // WiFi main thread
    static const int I2C_PRIORITY        = 17;  // I2C bus operations
    static const int RCIN_PRIO           = 16;  // RC input
    static const int WIFI_PRIO2          = 15;  // WiFi secondary

    // Low Priority Group (4-10): Background tasks
    static const int MONITOR_PRIO        = 10;  // Monitor thread (watchdog, lockup detection)
    static const int IO_PRIO             = 8;   // General I/O
    static const int STORAGE_PRIO        = 6;   // Storage operations

    static const int MONITOR_SS   = 1024*2;     // Monitor thread
    static const int TIMER_SS     = 1024*3;
    static const int MAIN_SS      = 1024*5;
    static const int RCIN_SS      = 1024*3;
    static const int RCOUT_SS     = 1024*1.5;
    static const int WIFI_SS1     = 1024*2.25;
    static const int WIFI_SS2     = 1024*2.25;
    static const int UART_SS      = 1024*2.25;
    static const int DEVICE_SS    = 1024*4;     // DEVICEBUS/s
    static const int IO_SS        = 1024*3.5;   // APM_IO
    static const int STORAGE_SS   = 1024*2;     // APM_STORAGE

private:
    AP_HAL::HAL::Callbacks *callbacks;
    AP_HAL::Proc _failsafe;

    AP_HAL::MemberProc _timer_proc[ESP32_SCHEDULER_MAX_TIMER_PROCS];
    uint8_t _num_timer_procs;

    AP_HAL::MemberProc _io_proc[ESP32_SCHEDULER_MAX_IO_PROCS];
    uint8_t _num_io_procs;

    static bool _initialized;

    tskTaskControlBlock* _main_task_handle;
    tskTaskControlBlock* _timer_task_handle;
    tskTaskControlBlock* _rcin_task_handle;
    tskTaskControlBlock* _rcout_task_handle;
    tskTaskControlBlock* _uart_task_handle;
    tskTaskControlBlock* _io_task_handle;
    tskTaskControlBlock* _storage_task_handle;
    tskTaskControlBlock* _monitor_task_handle;
    tskTaskControlBlock* test_task_handle;

    static void _main_thread(void *arg);
    static void _timer_thread(void *arg);
    static void _rcout_thread(void *arg);
    static void _rcin_thread(void *arg);
    static void _uart_thread(void *arg);
    static void _io_thread(void *arg);
    static void _storage_thread(void *arg);
    static void _monitor_thread(void *arg);

    static void set_position(void* arg);

    static void _print_profile(void* arg);

    static void test_esc(void* arg);

    static void wdt_init(uint32_t timeout, uint32_t core_mask);

    bool _in_timer_proc;
    void _run_timers();
    Semaphore _timer_sem;

    bool _in_io_proc;
    void _run_io();
    Semaphore _io_sem;

    // Priority boost support
    bool _priority_boosted;
    UBaseType_t _original_priority;
    uint64_t _boost_end_time_us;
    bool _called_boost;  // Track if boost was called in current loop

    // Expected delay support (suppress watchdog warnings)
    uint32_t _expect_delay_start;
    uint32_t _expect_delay_length;

    // Monitor thread support
    uint32_t last_watchdog_pat_ms;  // Last time watchdog was patted in main loop

public:
    // Check if delay_microseconds_boost() was called
    bool check_called_boost(void);

    // Stack monitoring
    void monitor_stack_usage(void);
    uint32_t get_stack_free(TaskHandle_t task);

    // Dynamic priority management (runtime adjustment)
    bool adjust_task_priority(TaskHandle_t task, int8_t priority_delta);
    UBaseType_t get_task_priority(TaskHandle_t task);

    // Deadlock detection
    struct TaskState {
        TaskHandle_t handle;
        eTaskState state;
        uint32_t runtime;
        uint32_t last_check_time;
    };
    void check_for_deadlock(void);

    // Task starvation monitoring
    void check_task_starvation(void);

private:
};
