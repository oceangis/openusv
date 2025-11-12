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

#include "AP_HAL_ESP32/Scheduler.h"
#include "AP_HAL_ESP32/RCInput.h"
#include "AP_HAL_ESP32/AnalogIn.h"
#include "AP_Math/AP_Math.h"
#include "SdCard.h"
#include "Profile.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_task_wdt.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <stdio.h>

//#define SCHEDULERDEBUG 1

using namespace ESP32;

extern const AP_HAL::HAL& hal;

bool Scheduler::_initialized = true;

Scheduler::Scheduler()
{
    _initialized = false;
    _priority_boosted = false;
    _original_priority = 0;
    _boost_end_time_us = 0;
    _called_boost = false;
    _expect_delay_start = 0;
    _expect_delay_length = 0;
    last_watchdog_pat_ms = 0;
}

Scheduler::~Scheduler()
{
    if (_initialized) {
        esp_task_wdt_deinit();
    }
}

void Scheduler::wdt_init(uint32_t timeout, uint32_t core_mask)
{
    esp_task_wdt_config_t config = {
        .timeout_ms = timeout,
        .idle_core_mask = core_mask,
        .trigger_panic = true
    };

    if ( ESP_OK != esp_task_wdt_init(&config) ) {
        printf("esp_task_wdt_init() failed\n");
    }

    if (ESP_OK != esp_task_wdt_add(NULL)) {
        printf("esp_task_wdt_add(NULL) failed");
    }
}

void Scheduler::init()
{

#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif

    hal.console->printf("%s:%d running with CONFIG_FREERTOS_HZ=%d\n", __PRETTY_FUNCTION__, __LINE__,CONFIG_FREERTOS_HZ);

    // keep main tasks that need speed on CPU 0
    // pin potentially slow stuff to CPU 1, as we have disabled the WDT on that core.
    #define FASTCPU 0
    #define SLOWCPU 1

    // pin main thread to Core 0, and we'll also pin other heavy-tasks to core 1, like wifi-related.
    if (xTaskCreatePinnedToCore(_main_thread, "APM_MAIN", Scheduler::MAIN_SS, this, Scheduler::MAIN_PRIO, &_main_task_handle,FASTCPU) != pdPASS) {
    //if (xTaskCreate(_main_thread, "APM_MAIN", Scheduler::MAIN_SS, this, Scheduler::MAIN_PRIO, &_main_task_handle) != pdPASS) {
        hal.console->printf("FAILED to create task _main_thread on FASTCPU\n");
    } else {
    	hal.console->printf("OK created task _main_thread on FASTCPU\n");
    }

    if (xTaskCreatePinnedToCore(_timer_thread, "APM_TIMER", TIMER_SS, this, TIMER_PRIO, &_timer_task_handle,FASTCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _timer_thread on FASTCPU\n");
    } else {
    	hal.console->printf("OK created task _timer_thread on FASTCPU\n");
    }	

    if (xTaskCreatePinnedToCore(_rcout_thread, "APM_RCOUT", RCOUT_SS, this, RCOUT_PRIO, &_rcout_task_handle,SLOWCPU) != pdPASS) {
       hal.console->printf("FAILED to create task _rcout_thread on SLOWCPU\n");
    } else {
       hal.console->printf("OK created task _rcout_thread on SLOWCPU\n");
    }

    if (xTaskCreatePinnedToCore(_rcin_thread, "APM_RCIN", RCIN_SS, this, RCIN_PRIO, &_rcin_task_handle,SLOWCPU) != pdPASS) {
       hal.console->printf("FAILED to create task _rcin_thread on SLOWCPU\n");
    } else {
       hal.console->printf("OK created task _rcin_thread on SLOWCPU\n");
    }

    // pin this thread to Core 1 as it keeps all teh uart/s feed data, and we need that quick.
    if (xTaskCreatePinnedToCore(_uart_thread, "APM_UART", UART_SS, this, UART_PRIO, &_uart_task_handle,FASTCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _uart_thread on FASTCPU\n");
    } else {
    	hal.console->printf("OK created task _uart_thread on FASTCPU\n");
    }	  

    // we put those on the SLOW core as it mounts the sd card, and that often isn't connected.
    if (xTaskCreatePinnedToCore(_io_thread, "SchedulerIO:APM_IO", IO_SS, this, IO_PRIO, &_io_task_handle,SLOWCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _io_thread on SLOWCPU\n");
    } else {
        hal.console->printf("OK created task _io_thread on SLOWCPU\n");
    }	 

    if (xTaskCreatePinnedToCore(_storage_thread, "APM_STORAGE", STORAGE_SS, this, STORAGE_PRIO, &_storage_task_handle,SLOWCPU) != pdPASS) { //no actual flash writes without this, storage kinda appears to work, but does an erase on every boot and params don't persist over reset etc.
        hal.console->printf("FAILED to create task _storage_thread on SLOWCPU\n");
    } else {
    	hal.console->printf("OK created task _storage_thread on SLOWCPU\n");
    }

    // Create monitor thread for lockup detection (runs on SLOWCPU to avoid interfering with critical tasks)
    if (xTaskCreatePinnedToCore(_monitor_thread, "APM_MONITOR", MONITOR_SS, this, MONITOR_PRIO, &_monitor_task_handle,SLOWCPU) != pdPASS) {
        hal.console->printf("FAILED to create task _monitor_thread on SLOWCPU\n");
    } else {
        hal.console->printf("OK created task _monitor_thread on SLOWCPU\n");
    }

    //   xTaskCreatePinnedToCore(_print_profile, "APM_PROFILE", IO_SS, this, IO_PRIO, nullptr,SLOWCPU);
}

template <typename T>
void executor(T oui)
{
    oui();
}

void IRAM_ATTR Scheduler::thread_create_trampoline(void *ctx)
{
    AP_HAL::MemberProc *t = (AP_HAL::MemberProc *)ctx;
    (*t)();
    free(t);

    // delete the calling task
    vTaskDelete(NULL);
}

/*
  create a new thread
*/
bool Scheduler::thread_create(AP_HAL::MemberProc proc, const char *name, uint32_t requested_stack_size, priority_base base, int8_t priority)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif

    // take a copy of the MemberProc, it is freed after thread exits
    AP_HAL::MemberProc *tproc = (AP_HAL::MemberProc *)calloc(1, sizeof(proc));
    if (!tproc) {
        return false;
    }
    *tproc = proc;

    uint8_t thread_priority = IO_PRIO;
    static const struct {
        priority_base base;
        uint8_t p;
    } priority_map[] = {
        { PRIORITY_BOOST, IO_PRIO},
        { PRIORITY_MAIN, MAIN_PRIO},
        { PRIORITY_SPI, SPI_PRIORITY},
        { PRIORITY_I2C, I2C_PRIORITY},
        { PRIORITY_CAN, IO_PRIO},
        { PRIORITY_TIMER, TIMER_PRIO},
        { PRIORITY_RCIN, RCIN_PRIO},
        { PRIORITY_IO, IO_PRIO},
        { PRIORITY_UART, UART_PRIO},
        { PRIORITY_NET, WIFI_PRIO1},
        { PRIORITY_STORAGE, STORAGE_PRIO},
        { PRIORITY_SCRIPTING, UART_PRIO},
    };
    for (uint8_t i=0; i<ARRAY_SIZE(priority_map); i++) {
        if (priority_map[i].base == base) {
#ifdef SCHEDDEBUG
            printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
            thread_priority = constrain_int16(priority_map[i].p + priority, 1, 25);
            break;
        }
    }
    // chibios has a 'thread working area', we just another 1k.
    #define EXTRA_THREAD_SPACE 1024
    uint32_t actual_stack_size = requested_stack_size+EXTRA_THREAD_SPACE;

    tskTaskControlBlock* xhandle;
    BaseType_t xReturned = xTaskCreate(thread_create_trampoline, name, actual_stack_size, tproc, thread_priority, &xhandle);
    if (xReturned != pdPASS) {
        free(tproc);
        return false;
    }
    return true;
}

void IRAM_ATTR Scheduler::delay(uint16_t ms)
{
    uint64_t start = AP_HAL::micros64();
    while ((AP_HAL::micros64() - start)/1000 < ms) {
        delay_microseconds(1000);
        if (_min_delay_cb_ms <= ms) {
            if (in_main_thread()) {
                call_delay_cb();
            }
        }
    }
}

void IRAM_ATTR Scheduler::delay_microseconds(uint16_t us)
{
    if (in_main_thread() && us < 100) {
        esp_rom_delay_us(us);
    } else { // Minimum delay for FreeRTOS is 1ms
        uint32_t tick = portTICK_PERIOD_MS * 1000;

        vTaskDelay((us+tick-1)/tick);
    }
}

/*
  Delay with priority boost
  Based on ChibiOS implementation - boosts main thread priority
  to ensure it runs immediately after the delay
 */
void IRAM_ATTR Scheduler::delay_microseconds_boost(uint16_t us)
{
    if (!in_main_thread()) {
        // Only boost main thread
        delay_microseconds(us);
        return;
    }

    if (!_priority_boosted) {
        // Save current priority and boost
        TaskHandle_t current_task = xTaskGetCurrentTaskHandle();
        _original_priority = uxTaskPriorityGet(current_task);

        // Boost to MAIN_PRIO_BOOST (modest increase, not maximum)
        vTaskPrioritySet(current_task, MAIN_PRIO_BOOST);
        _priority_boosted = true;
        _called_boost = true;  // Mark that boost was called
    }

    // Perform the delay
    delay_microseconds(us);
}

/*
  End priority boost
  Called explicitly or automatically
 */
void IRAM_ATTR Scheduler::boost_end(void)
{
    if (!_priority_boosted || !in_main_thread()) {
        return;
    }

    TaskHandle_t current_task = xTaskGetCurrentTaskHandle();

    // Restore original priority
    vTaskPrioritySet(current_task, _original_priority);
    _priority_boosted = false;
}

/*
  Check if delay_microseconds_boost() was called since last check
  Used to determine if we should yield to lower priority tasks
 */
bool Scheduler::check_called_boost(void)
{
    if (!_called_boost) {
        return false;
    }
    _called_boost = false;
    return true;
}

void IRAM_ATTR Scheduler::register_timer_process(AP_HAL::MemberProc proc)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    for (uint8_t i = 0; i < _num_timer_procs; i++) {
        if (_timer_proc[i] == proc) {
            return;
        }
    }
    if (_num_timer_procs >= ESP32_SCHEDULER_MAX_TIMER_PROCS) {
        printf("Out of timer processes\n");
        return;
    }
    _timer_sem.take_blocking();
    _timer_proc[_num_timer_procs] = proc;
    _num_timer_procs++;
    _timer_sem.give();
}

void IRAM_ATTR Scheduler::register_io_process(AP_HAL::MemberProc proc)
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _io_sem.take_blocking();
    for (uint8_t i = 0; i < _num_io_procs; i++) {
        if (_io_proc[i] == proc) {
            _io_sem.give();
            return;
        }
    }
    if (_num_io_procs < ESP32_SCHEDULER_MAX_IO_PROCS) {
        _io_proc[_num_io_procs] = proc;
        _num_io_procs++;
    } else {
        printf("Out of IO processes\n");
    }
    _io_sem.give();
}

void IRAM_ATTR Scheduler::register_timer_failsafe(AP_HAL::Proc failsafe, uint32_t period_us)
{
    _failsafe = failsafe;
}

void Scheduler::reboot(bool hold_in_bootloader)
{
    printf("Restarting now...\n");
    hal.rcout->force_safety_on();
    unmount_sdcard();
    esp_restart();
}

bool IRAM_ATTR Scheduler::in_main_thread() const
{
    return _main_task_handle == xTaskGetCurrentTaskHandle();
}

void Scheduler::set_system_initialized()
{
#ifdef SCHEDDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_initialized) {
        AP_HAL::panic("PANIC: scheduler::system_initialized called more than once");
    }

    _initialized = true;
}

bool Scheduler::is_system_initialized()
{
    return _initialized;
}

void IRAM_ATTR Scheduler::_timer_thread(void *arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;

#if HAL_INS_DEFAULT != HAL_INS_NONE
    // wait to ensure INS system inits unless using HAL_INS_NONE
    while (!_initialized) {
        sched->delay_microseconds(1000);
    }
#endif

#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        sched->_run_timers();
        //analog in
#if AP_HAL_ANALOGIN_ENABLED
        ((AnalogIn*)hal.analogin)->_timer_tick();
#endif
    }
}

void IRAM_ATTR Scheduler::_rcout_thread(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(1000);
    }

    while (true) {
        sched->delay_microseconds(4000);
        // process any pending RC output requests
        hal.rcout->timer_tick();
    }
}

void IRAM_ATTR Scheduler::_run_timers()
{
#ifdef SCHEDULERDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_in_timer_proc) {
        return;
    }
#ifdef SCHEDULERDEBUG
    printf("%s:%d _in_timer_proc \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _in_timer_proc = true;

    int num_procs = 0;

    _timer_sem.take_blocking();
    num_procs = _num_timer_procs;
    _timer_sem.give();

    // now call the timer based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_timer_proc[i]) {
            _timer_proc[i]();
        }
    }

    // and the failsafe, if one is setup
    if (_failsafe != nullptr) {
        _failsafe();
    }

    _in_timer_proc = false;
}

void IRAM_ATTR Scheduler::_rcin_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!_initialized) {
        sched->delay_microseconds(20000);
    }
    hal.rcin->init();
    while (true) {
        sched->delay_microseconds(1000);
        ((RCInput *)hal.rcin)->_timer_tick();
    }
}

void IRAM_ATTR Scheduler::_run_io(void)
{
#ifdef SCHEDULERDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_in_io_proc) {
        return;
    }
#ifdef SCHEDULERDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _in_io_proc = true;

    int num_procs = 0;
    _io_sem.take_blocking();
    num_procs = _num_io_procs;
    _io_sem.give();
    // now call the IO based drivers
    for (int i = 0; i < num_procs; i++) {
        if (_io_proc[i]) {
            _io_proc[i]();
        }
    }
    _in_io_proc = false;
}

void IRAM_ATTR Scheduler::_io_thread(void* arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    mount_sdcard();
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(1000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint32_t last_sd_start_ms = AP_HAL::millis();
    while (true) {
        sched->delay_microseconds(1000);
        // run registered IO processes
        sched->_run_io();

        if (!hal.util->get_soft_armed()) {
            // if sdcard hasn't mounted then retry it every 3s in the IO
            // thread when disarmed
            uint32_t now = AP_HAL::millis();
            if (now - last_sd_start_ms > 3000) {
                last_sd_start_ms = now;
                sdcard_retry();
            }
        }
    }
}


void Scheduler::_storage_thread(void* arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(10000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        // process any pending storage writes
        hal.storage->_timer_tick();
        //print_profile();
    }
}

void Scheduler::_print_profile(void* arg)
{
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(10000);
    }

    while (true) {
        sched->delay(10000);
        print_profile();
    }

}

void IRAM_ATTR Scheduler::_uart_thread(void *arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;
    while (!sched->_initialized) {
        sched->delay_microseconds(2000);
    }
#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->delay_microseconds(1000);
        for (uint8_t i=0; i<hal.num_serial; i++) {
            hal.serial(i)->_timer_tick();
        }
        hal.console->_timer_tick();
    }
}


// get the active main loop rate
uint16_t IRAM_ATTR Scheduler::get_loop_rate_hz(void)
{
    if (_active_loop_rate_hz == 0) {
        _active_loop_rate_hz = _loop_rate_hz;
    }
    return _active_loop_rate_hz;
}

/*
  Get stack free space for a task
*/
uint32_t Scheduler::get_stack_free(TaskHandle_t task)
{
    if (task == nullptr) {
        return 0;
    }
    return uxTaskGetStackHighWaterMark(task);
}

/*
  Monitor stack usage of all threads
  Warns if any thread is using more than 80% of its stack
*/
void Scheduler::monitor_stack_usage(void)
{
    struct {
        TaskHandle_t* handle;
        const char* name;
        uint32_t stack_size;
    } tasks[] = {
        {&_main_task_handle, "Main", MAIN_SS},
        {&_timer_task_handle, "Timer", TIMER_SS},
        {&_rcin_task_handle, "RCIn", RCIN_SS},
        {&_rcout_task_handle, "RCOut", RCOUT_SS},
        {&_uart_task_handle, "UART", UART_SS},
        {&_io_task_handle, "IO", IO_SS},
        {&_storage_task_handle, "Storage", STORAGE_SS},
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(tasks); i++) {
        if (tasks[i].handle && *tasks[i].handle) {
            uint32_t free_stack = get_stack_free(*tasks[i].handle);
            uint32_t used_stack = tasks[i].stack_size - free_stack;
            uint8_t usage_percent = (used_stack * 100) / tasks[i].stack_size;

            if (usage_percent > 80) {
                printf("WARNING: %s thread stack usage: %u%% (%u/%u bytes)\n",
                       tasks[i].name, usage_percent, used_stack, tasks[i].stack_size);
            }
        }
    }
}

// once every 60 seconds, print some stats...
void Scheduler::print_stats(void)
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 60000) {
        char buffer[1024];
        vTaskGetRunTimeStats(buffer);
        printf("\n\n%s\n", buffer);
        heap_caps_print_heap_info(0);

        // Monitor stack usage
        monitor_stack_usage();

        last_run = AP_HAL::millis64();
    }

    // printf("loop_rate_hz: %d",get_loop_rate_hz());
}

// Run every 10s
void Scheduler::print_main_loop_rate(void)
{
    static int64_t last_run = 0;
    if (AP_HAL::millis64() - last_run > 10000) {
        last_run = AP_HAL::millis64();
        // null pointer in here...
        const float actual_loop_rate = AP::scheduler().get_filtered_loop_rate_hz();
        const uint16_t expected_loop_rate = AP::scheduler().get_loop_rate_hz();
        hal.console->printf("loop_rate: actual: %fHz, expected: %uHz\n", actual_loop_rate, expected_loop_rate);
    }
}

void IRAM_ATTR Scheduler::_main_thread(void *arg)
{
#ifdef SCHEDDEBUG
    printf("%s:%d start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    Scheduler *sched = (Scheduler *)arg;

#if AP_HAL_ANALOGIN_ENABLED
    hal.analogin->init();
#endif
    hal.rcout->init();

    sched->callbacks->setup();

    sched->set_system_initialized();

    //initialize WTD for current thread on FASTCPU, all cores will be (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1
    wdt_init( TWDT_TIMEOUT_MS, 1 << FASTCPU ); // 3 sec


#ifdef SCHEDDEBUG
    printf("%s:%d initialised\n", __PRETTY_FUNCTION__, __LINE__);
#endif
    while (true) {
        sched->callbacks->loop();

        /*
          Give up 50 microseconds of time if the loop hasn't
          called delay_microseconds_boost(), to ensure low priority
          drivers get a chance to run. Calling delay_microseconds_boost()
          means we have already given up time from the main loop.
         */
        if (!sched->check_called_boost()) {
            sched->delay_microseconds(50);
        }

        sched->delay_microseconds(250);

        // Update watchdog pat time for monitor thread
        sched->last_watchdog_pat_ms = AP_HAL::millis();

        // run stats periodically
#ifdef SCHEDDEBUG
        sched->print_stats();
#endif
        sched->print_main_loop_rate();

        if (ESP_OK != esp_task_wdt_reset()) {
            printf("esp_task_wdt_reset() failed\n");
        };
    }
}

/*
  Monitor thread - detects software lockups and main loop delays
  Based on ChibiOS implementation, adapted for ESP32/FreeRTOS
*/
void IRAM_ATTR Scheduler::_monitor_thread(void *arg)
{
    Scheduler *sched = (Scheduler *)arg;

    // Wait for system initialization
    while (!sched->_initialized) {
        sched->delay(100);
    }

    hal.console->printf("Monitor thread started\n");

    while (true) {
        sched->delay(100);  // Check every 100ms

        uint32_t now = AP_HAL::millis();
        uint32_t loop_delay = now - sched->last_watchdog_pat_ms;

        // Check for main loop stuck
        if (loop_delay >= 200 && loop_delay < 500) {
            // Main loop stuck for 200-500ms - warning level
            hal.console->printf("WARNING: Main loop delay %ums\n", (unsigned)loop_delay);
        }
        else if (loop_delay >= 500 && !sched->in_expected_delay()) {
            // Main loop stuck for >500ms and not in expected delay - critical
            hal.console->printf("CRITICAL: Main loop stuck %ums!\n", (unsigned)loop_delay);

            // Log to help debug
            hal.console->printf("  System state:\n");
            hal.console->printf("  - Free heap: %u bytes\n", (unsigned)hal.util->available_memory());

            // Try to recover by logging stack info
            sched->monitor_stack_usage();

            // Check for deadlock in critical tasks
            sched->check_for_deadlock();
        }

        // Periodic checks (every 5 seconds via internal rate limiting)
        sched->check_task_starvation();

        // Additional watchdog safety - pat the watchdog from monitor thread if main is stuck
        // This prevents full system reset while we're trying to diagnose
        if (loop_delay > 1000) {
            // Main loop completely stuck, pat watchdog to prevent reset during diagnosis
            esp_task_wdt_reset();
        }
    }
}

/*
  Set expected delay - used to suppress watchdog warnings during known long operations
*/
void Scheduler::expect_delay_ms(uint32_t ms)
{
    if (ms == 0) {
        _expect_delay_length = 0;
        _expect_delay_start = 0;
    } else {
        _expect_delay_start = AP_HAL::millis();
        _expect_delay_length = ms;
    }
}

/*
  Check if we are in an expected delay period
*/
bool Scheduler::in_expected_delay(void) const
{
    if (_expect_delay_length == 0) {
        return false;
    }
    uint32_t now = AP_HAL::millis();
    if (now - _expect_delay_start > _expect_delay_length) {
        return false;
    }
    return true;
}

/*
  Dynamic priority management - adjust task priority at runtime
  This allows adaptive priority control based on system load
*/
bool Scheduler::adjust_task_priority(TaskHandle_t task, int8_t priority_delta)
{
    if (task == nullptr) {
        return false;
    }

    UBaseType_t current_priority = uxTaskPriorityGet(task);
    int new_priority = (int)current_priority + priority_delta;

    // Clamp priority to valid range (1-24, avoid idle=0 and max=25)
    new_priority = constrain_int16(new_priority, 1, configMAX_PRIORITIES - 1);

    vTaskPrioritySet(task, (UBaseType_t)new_priority);
    return true;
}

/*
  Get task priority
*/
UBaseType_t Scheduler::get_task_priority(TaskHandle_t task)
{
    if (task == nullptr) {
        return 0;
    }
    return uxTaskPriorityGet(task);
}

/*
  Deadlock detection - check if any critical tasks are blocked
  Based on FreeRTOS task states
*/
void Scheduler::check_for_deadlock(void)
{
    struct {
        TaskHandle_t* handle;
        const char* name;
        bool critical;  // Critical tasks must not be blocked long
    } tasks[] = {
        {&_main_task_handle, "Main", true},
        {&_timer_task_handle, "Timer", true},
        {&_uart_task_handle, "UART", false},
        {&_rcin_task_handle, "RCIn", false},
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(tasks); i++) {
        if (tasks[i].handle && *tasks[i].handle && tasks[i].critical) {
            eTaskState state = eTaskGetState(*tasks[i].handle);

            // Blocked or suspended critical tasks indicate potential deadlock
            if (state == eBlocked || state == eSuspended) {
                hal.console->printf("WARNING: Critical task %s is %s\n",
                                  tasks[i].name,
                                  state == eBlocked ? "blocked" : "suspended");
            }
        }
    }
}

/*
  Task starvation monitoring - detect tasks not getting CPU time
  Uses FreeRTOS runtime stats to track CPU usage
*/
void Scheduler::check_task_starvation(void)
{
    static uint32_t last_check_time = 0;
    static uint32_t last_runtime[10] = {0};  // Track runtime for up to 10 tasks

    uint32_t now = AP_HAL::millis();
    if (now - last_check_time < 5000) {  // Check every 5 seconds
        return;
    }
    last_check_time = now;

    struct {
        TaskHandle_t* handle;
        const char* name;
        uint8_t index;
    } tasks[] = {
        {&_main_task_handle, "Main", 0},
        {&_timer_task_handle, "Timer", 1},
        {&_uart_task_handle, "UART", 2},
        {&_io_task_handle, "IO", 3},
        {&_storage_task_handle, "Storage", 4},
    };

    for (uint8_t i = 0; i < ARRAY_SIZE(tasks); i++) {
        if (tasks[i].handle && *tasks[i].handle) {
            TaskStatus_t task_status;
            vTaskGetInfo(*tasks[i].handle, &task_status, pdTRUE, eInvalid);

            uint32_t runtime_delta = task_status.ulRunTimeCounter - last_runtime[tasks[i].index];
            last_runtime[tasks[i].index] = task_status.ulRunTimeCounter;

            // If runtime hasn't increased, task might be starved
            if (runtime_delta == 0 && task_status.eCurrentState == eReady) {
                hal.console->printf("WARNING: Task %s may be starved (ready but not running)\n",
                                  tasks[i].name);
            }
        }
    }
}

