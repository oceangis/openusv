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
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit and David "Buzz" Bussenschutt
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "Util.h"

#include "RCOutput.h"

#include <AP_ROMFS/AP_ROMFS.h>
#include "SdCard.h"

#include <esp_timer.h>
#include <multi_heap.h>
#include <esp_heap_caps.h>

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include <AP_Common/ExpandingString.h>

#include "esp_mac.h"
#include "esp_random.h"
#include "soc/rtc.h"
#include "esp_private/esp_clk.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if HAL_UART_STATS_ENABLED
#include "UARTDriver.h"
#endif

extern const AP_HAL::HAL& hal;

using namespace ESP32;


/**
   how much free memory do we have in bytes.
*/
uint32_t Util::available_memory(void)
{
    return heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);

}

/*
    Special Allocation Routines
*/

void* Util::malloc_type(size_t size, AP_HAL::Util::Memory_Type mem_type)
{

    // https://docs.espressif.com/projects/esp-idf/en/v4.0.2/api-reference/system/mem_alloc.html
    // esp32 has DRAM, IRAM and D/IRAM that can be used as either

    /*
    DRAM (Data RAM) is memory used to hold data. This is the most common kind of memory accessed as heap.

    IRAM (Instruction RAM) usually holds executable data only. If accessed as generic memory, all accesses must be 32-bit aligned.

    D/IRAM is RAM which can be used as either Instruction or Data RAM.
    */

    //The ESP-IDF malloc() implementation internally calls heap_caps_malloc(size, MALLOC_CAP_8BIT) in order to allocate DRAM that is byte-addressable.

    //For most purposes, the standard libc malloc() and free() functions can be used for heap allocation without any special consideration.
    //	return malloc(size);

    if (mem_type == AP_HAL::Util::MEM_DMA_SAFE) {
        return heap_caps_calloc(1, size, MALLOC_CAP_DMA);
        //} else if (mem_type == AP_HAL::Util::MEM_FAST) {
        //   return heap_caps_calloc(1, size, MALLOC_CAP_32BIT); //WARNING 32bit memory cannot use unless 32bit access
    } else {
        return heap_caps_calloc(1, size, MALLOC_CAP_8BIT);
    }
}

void Util::free_type(void *ptr, size_t size, AP_HAL::Util::Memory_Type mem_type)
{
    if (ptr != NULL) {
        heap_caps_free(ptr);
    }
}


/*
  get safety switch state
 */
Util::safety_state Util::safety_switch_state(void)
{

#if HAL_USE_PWM == TRUE
    return ((RCOutput *)hal.rcout)->_safety_switch_state();
#else
    return SAFETY_NONE;
#endif
}

#ifdef HAL_PWM_ALARM
struct Util::ToneAlarmPwmGroup Util::_toneAlarm_pwm_group = HAL_PWM_ALARM;

bool Util::toneAlarm_init()
{
    _toneAlarm_pwm_group.pwm_cfg.period = 1000;
    pwmStart(_toneAlarm_pwm_group.pwm_drv, &_toneAlarm_pwm_group.pwm_cfg);

    return true;
}

void Util::toneAlarm_set_buzzer_tone(float frequency, float volume, uint32_t duration_ms)
{
    if (is_zero(frequency) || is_zero(volume)) {
        pwmDisableChannel(_toneAlarm_pwm_group.pwm_drv, _toneAlarm_pwm_group.chan);
    } else {
        pwmChangePeriod(_toneAlarm_pwm_group.pwm_drv,
                        roundf(_toneAlarm_pwm_group.pwm_cfg.frequency/frequency));

        pwmEnableChannel(_toneAlarm_pwm_group.pwm_drv, _toneAlarm_pwm_group.chan, roundf(volume*_toneAlarm_pwm_group.pwm_cfg.frequency/frequency)/2);
    }
}
#endif // HAL_PWM_ALARM

/*
  set HW RTC in UTC microseconds
*/
void Util::set_hw_rtc(uint64_t time_utc_usec)
{
    //stm32_set_utc_usec(time_utc_usec);
}

/*
  get system clock in UTC microseconds
*/
uint64_t Util::get_hw_rtc() const
{
    return esp_timer_get_time();
}

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)

#if !HAL_GCS_ENABLED
#define Debug(fmt, args ...)  do { hal.console->printf(fmt, ## args); } while (0)
#else
#include <GCS_MAVLink/GCS.h>
#define Debug(fmt, args ...)  do { GCS_SEND_TEXT(MAV_SEVERITY_INFO, fmt, ## args); } while (0)
#endif

Util::FlashBootloader Util::flash_bootloader()
{
    //    ....esp32 too
    return FlashBootloader::FAIL;
}
#endif // !HAL_NO_FLASH_SUPPORT && !HAL_NO_ROMFS_SUPPORT

/*
  display system identifier - board type and serial number
 */


bool Util::get_system_id(char buf[50])
{
    //uint8_t serialid[12];
    char board_name[] = HAL_ESP32_BOARD_NAME" ";

    uint8_t base_mac_addr[6] = {0};
    esp_err_t ret = esp_efuse_mac_get_custom(base_mac_addr);
    if (ret != ESP_OK) {
        ret = esp_efuse_mac_get_default(base_mac_addr);
    }

    char board_mac[20] = "                   ";
    snprintf(board_mac,20, "%x %x %x %x %x %x",
             base_mac_addr[0], base_mac_addr[1], base_mac_addr[2], base_mac_addr[3], base_mac_addr[4], base_mac_addr[5]);

    // null terminate both
    //board_name[13] = 0;
    board_mac[19] = 0;

    // tack strings together
    snprintf(buf, 40, "%s %s", board_name, board_mac);
    // and null terminate that too..
    buf[39] = 0;
    return true;
}

bool Util::get_system_id_unformatted(uint8_t buf[], uint8_t &len)
{
    uint8_t base_mac_addr[6] = {0};
    esp_err_t ret = esp_efuse_mac_get_custom(base_mac_addr);
    if (ret != ESP_OK) {
        ret = esp_efuse_mac_get_default(base_mac_addr);
    }

    len = MIN(len, ARRAY_SIZE(base_mac_addr));
    memcpy(buf, (const void *)base_mac_addr, len);

    return true;
}

// return true if the reason for the reboot was a watchdog reset
bool Util::was_watchdog_reset() const
{
    return false;
    esp_reset_reason_t reason = esp_reset_reason();

    return reason == ESP_RST_PANIC
           || reason == ESP_RST_PANIC
           || reason == ESP_RST_TASK_WDT
           || reason == ESP_RST_WDT;
}

/*
  display stack usage as text buffer for @SYS/threads.txt
 */
void Util::thread_info(ExpandingString &str)
{
    // a header to allow for machine parsers to determine format
    str.printf("ThreadsV1\n");

    // Get number of tasks
    UBaseType_t task_count = uxTaskGetNumberOfTasks();

    // Allocate array for task status
    TaskStatus_t *task_status_array = (TaskStatus_t *)malloc(task_count * sizeof(TaskStatus_t));
    if (task_status_array == nullptr) {
        return;
    }

    // Get task information
    uint32_t total_runtime;
    task_count = uxTaskGetSystemState(task_status_array, task_count, &total_runtime);

    // Print header
    str.printf("%-20s %8s %8s %8s %4s\n", "Name", "Stack", "Priority", "Runtime", "Core");

    // Print each task
    for (UBaseType_t i = 0; i < task_count; i++) {
        TaskStatus_t *task = &task_status_array[i];

        // Calculate stack usage
        uint32_t stack_free = task->usStackHighWaterMark;

        // Get core affinity (ESP32 specific)
        int core = -1;
#if CONFIG_FREERTOS_UNICORE
        core = 0;
#else
        BaseType_t task_core = xTaskGetAffinity(task->xHandle);
        if (task_core == tskNO_AFFINITY) {
            core = -1;
        } else {
            core = task_core;
        }
#endif

        str.printf("%-20s %8lu %8lu %8lu %4d\n",
                   task->pcTaskName,
                   (unsigned long)stack_free,
                   (unsigned long)task->uxCurrentPriority,
                   (unsigned long)task->ulRunTimeCounter,
                   core);
    }

    free(task_status_array);
}

/*
  Display DMA contention information
 */
void Util::dma_info(ExpandingString &str)
{
    str.printf("DMA Info:\n");
    str.printf("ESP32 uses internal DMA controllers\n");

    // Get DMA-capable memory info
    size_t dma_free = heap_caps_get_free_size(MALLOC_CAP_DMA);
    size_t dma_total = heap_caps_get_total_size(MALLOC_CAP_DMA);
    size_t dma_largest = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);

    str.printf("DMA Memory: free=%lu total=%lu largest_block=%lu\n",
               (unsigned long)dma_free,
               (unsigned long)dma_total,
               (unsigned long)dma_largest);
}

/*
  Display memory allocation information
 */
void Util::mem_info(ExpandingString &str)
{
    str.printf("Memory Info:\n");

    // Get heap info for different memory types
    multi_heap_info_t heap_info;

    // Default heap (MALLOC_CAP_8BIT)
    heap_caps_get_info(&heap_info, MALLOC_CAP_8BIT);
    str.printf("8-bit capable: free=%lu allocated=%lu largest_block=%lu\n",
               (unsigned long)heap_info.total_free_bytes,
               (unsigned long)heap_info.total_allocated_bytes,
               (unsigned long)heap_info.largest_free_block);

    // 32-bit aligned memory
    heap_caps_get_info(&heap_info, MALLOC_CAP_32BIT);
    str.printf("32-bit capable: free=%lu allocated=%lu largest_block=%lu\n",
               (unsigned long)heap_info.total_free_bytes,
               (unsigned long)heap_info.total_allocated_bytes,
               (unsigned long)heap_info.largest_free_block);

    // DMA-capable memory
    heap_caps_get_info(&heap_info, MALLOC_CAP_DMA);
    str.printf("DMA capable: free=%lu allocated=%lu largest_block=%lu\n",
               (unsigned long)heap_info.total_free_bytes,
               (unsigned long)heap_info.total_allocated_bytes,
               (unsigned long)heap_info.largest_free_block);

    // SPIRAM (external RAM)
#if CONFIG_ESP32_SPIRAM_SUPPORT || CONFIG_ESP32S3_SPIRAM_SUPPORT
    heap_caps_get_info(&heap_info, MALLOC_CAP_SPIRAM);
    str.printf("SPIRAM: free=%lu allocated=%lu largest_block=%lu\n",
               (unsigned long)heap_info.total_free_bytes,
               (unsigned long)heap_info.total_allocated_bytes,
               (unsigned long)heap_info.largest_free_block);
#endif
}

/*
  Display timer frequency information
 */
void Util::timer_info(ExpandingString &str)
{
    str.printf("Timer Info:\n");
    str.printf("CPU Frequency: %lu MHz\n", (unsigned long)(esp_clk_cpu_freq() / 1000000));
    str.printf("APB Frequency: %lu MHz\n", (unsigned long)(rtc_clk_apb_freq_get() / 1000000));
    str.printf("RTC Slow Clock: %lu Hz\n", (unsigned long)rtc_clk_slow_freq_get_hz());
    str.printf("RTC Fast Clock: %lu Hz\n", (unsigned long)rtc_clk_fast_freq_get());
}

/*
  Generate random values using ESP32 hardware RNG
 */
bool Util::get_random_vals(uint8_t* data, size_t size)
{
    if (data == nullptr || size == 0) {
        return false;
    }

    // ESP32 hardware RNG
    esp_fill_random(data, size);
    return true;
}

/*
  Generate true random values with timeout
  ESP32's RNG is hardware-based and doesn't need blocking
 */
bool Util::get_true_random_vals(uint8_t* data, size_t size, uint32_t timeout_us)
{
    if (data == nullptr || size == 0) {
        return false;
    }

    // ESP32's hardware RNG is fast enough, no need for timeout handling
    esp_fill_random(data, size);
    return true;
}

#if HAL_UART_STATS_ENABLED
/*
  Display UART statistics
 */
void Util::uart_info(ExpandingString &str)
{
    str.printf("UART Statistics:\n");

    // Iterate through all UART ports
    extern ESP32::UARTDesc uart_desc[];

    for (uint8_t i = 0; i < hal.num_serial; i++) {
        UARTDriver *uart = (UARTDriver *)hal.serial(i);
        if (uart != nullptr && uart->is_initialized()) {
            UARTDriver::StatsTracker stats;
            uint32_t dt_ms = 1000; // 1 second interval for rate calculation
            uart->uart_info(str, stats, dt_ms);
        }
    }
}

#if HAL_LOGGING_ENABLED
/*
  Log UART statistics to dataflash
 */
void Util::uart_log()
{
    // This would integrate with AP_Logger
    // For now, it's a placeholder for future implementation
    // Actual logging would require access to AP_Logger instance
}
#endif // HAL_LOGGING_ENABLED
#endif // HAL_UART_STATS_ENABLED

/*
  Get system load (CPU utilization)
 */
bool Util::get_system_load(float& avg_load, float& peak_load) const
{
    // FreeRTOS doesn't directly provide CPU load
    // We can calculate it from idle task runtime

    TaskStatus_t *task_status_array;
    UBaseType_t task_count = uxTaskGetNumberOfTasks();

    task_status_array = (TaskStatus_t *)malloc(task_count * sizeof(TaskStatus_t));
    if (task_status_array == nullptr) {
        return false;
    }

    uint32_t total_runtime;
    task_count = uxTaskGetSystemState(task_status_array, task_count, &total_runtime);

    // Find IDLE tasks runtime
    uint32_t idle_runtime = 0;
    for (UBaseType_t i = 0; i < task_count; i++) {
        if (strncmp(task_status_array[i].pcTaskName, "IDLE", 4) == 0) {
            idle_runtime += task_status_array[i].ulRunTimeCounter;
        }
    }

    free(task_status_array);

    if (total_runtime == 0) {
        return false;
    }

    // Calculate CPU load (100% - idle%)
    float idle_percent = (idle_runtime * 100.0f) / total_runtime;
    float cpu_load = 100.0f - idle_percent;

    // For now, set both avg and peak to current load
    // A proper implementation would track these over time
    avg_load = cpu_load;
    peak_load = cpu_load;

    return true;
}


