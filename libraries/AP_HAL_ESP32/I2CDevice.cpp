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
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_ESP32/Semaphores.h>
#include "Scheduler.h"
#include "driver/gpio.h"

using namespace ESP32;

extern const AP_HAL::HAL& hal;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

I2CBusDesc i2c_bus_desc[] = { HAL_ESP32_I2C_BUSES };

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(i2c_bus_desc)];

I2CDeviceManager::I2CDeviceManager(void)
{
    for (uint8_t i=0; i<ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].soft) {
            businfo[i].sw_handle.sda = i2c_bus_desc[i].sda;
            businfo[i].sw_handle.scl = i2c_bus_desc[i].scl;
            //TODO make modular
            businfo[i].sw_handle.speed = I2C_SPEED_FAST;
            businfo[i].soft = true;
            businfo[i].sda_pin = i2c_bus_desc[i].sda;
            businfo[i].scl_pin = i2c_bus_desc[i].scl;
            i2c_init(&(businfo[i].sw_handle));
        } else {
            i2c_config_t i2c_bus_config;
            i2c_bus_config.mode = I2C_MODE_MASTER;
            i2c_bus_config.sda_io_num = i2c_bus_desc[i].sda;
            i2c_bus_config.scl_io_num = i2c_bus_desc[i].scl;
            i2c_bus_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
            i2c_bus_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
            i2c_bus_config.master.clk_speed = i2c_bus_desc[i].speed;
            i2c_bus_config.clk_flags = 0;
            i2c_port_t p = i2c_bus_desc[i].port;
            businfo[i].port = p;
            businfo[i].bus_clock = i2c_bus_desc[i].speed;
            businfo[i].soft = false;
            businfo[i].sda_pin = i2c_bus_desc[i].sda;
            businfo[i].scl_pin = i2c_bus_desc[i].scl;
            i2c_param_config(p, &i2c_bus_config);
            i2c_driver_install(p, I2C_MODE_MASTER, 0, 0, ESP_INTR_FLAG_IRAM);
            i2c_filter_enable(p, 7);
        }
    }
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    bus(I2CDeviceManager::businfo[busnum]),
    _retries(10),
    _address(address),
    _timeout_ms(timeout_ms)
{
    set_device_bus(busnum);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x",
             (unsigned)busnum, (unsigned)address);
}

I2CDevice::~I2CDevice()
{
    free(pname);
}

#if HAL_I2C_CLEAR_ON_TIMEOUT
/*
  Clear a stuck I2C bus by clocking out pulses on SCL.
  This allows a device holding SDA low to complete its transaction.
  SAFE METHOD: Only manipulates GPIO, does NOT delete/reinstall I2C driver.
*/
void I2CBus::clear_bus(void)
{
    if (soft) {
        // Software I2C doesn't need bus recovery
        return;
    }

    // Temporarily set SCL as GPIO output
    gpio_set_direction(scl_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(scl_pin, GPIO_PULLUP_ONLY);

    // Toggle SCL 20 times to allow device to complete transaction
    for (uint8_t i = 0; i < 20; i++) {
        gpio_set_level(scl_pin, 0);
        hal.scheduler->delay_microseconds(10);
        gpio_set_level(scl_pin, 1);
        hal.scheduler->delay_microseconds(10);
    }

    // I2C driver will automatically reconfigure pins on next transaction
    // No need to manually restore - this is the key difference from old code!
}

/*
  Read SDA state to check if bus is stuck
*/
uint8_t I2CBus::read_sda(void)
{
    if (soft) {
        return 1;  // Assume OK for software I2C
    }

    // Temporarily set as input to read the pin state
    gpio_set_direction(sda_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(sda_pin, GPIO_PULLUP_ONLY);

    uint8_t level = gpio_get_level(sda_pin);

    // Pin will be reconfigured by I2C driver on next transaction
    return level;
}

/*
  Clear all I2C buses
*/
void I2CBus::clear_all(void)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(i2c_bus_desc); i++) {
        I2CDeviceManager::businfo[i].clear_bus();
    }
}
#endif // HAL_I2C_CLEAR_ON_TIMEOUT

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    // Boundary checks
#if HAL_I2C_INTERNAL_CHECKS
    if ((send_len > 0 && send == nullptr) || (recv_len > 0 && recv == nullptr)) {
        printf("I2C: Invalid buffer pointers (addr=0x%02x)\n", _address);
        return false;
    }
    if (send_len + recv_len > 256) {
        printf("I2C: Transfer too large (addr=0x%02x, len=%u)\n",
               _address, send_len + recv_len);
        return false;
    }
#endif

    if (!bus.semaphore.check_owner()) {
        printf("I2C: Semaphore not owned (addr=0x%02x)\n", _address);
        return false;
    }

    bool result = false;
    if (bus.soft) {
        uint8_t flag_wr = (recv_len == 0 || recv == nullptr) ? I2C_NOSTOP : 0;
        if (send_len != 0 && send != nullptr) {
            //tx with optional rx (after tx)
            i2c_write_bytes(&bus.sw_handle,
                            _address,
                            send,
                            send_len,
                            flag_wr);
        }
        if (recv_len != 0 && recv != nullptr) {
            //rx only or rx after tx
            //rx separated from tx by (re)start
            i2c_read_bytes(&bus.sw_handle,
                           _address,
                           (uint8_t *)recv, recv_len, 0);
        }
        result = true; //TODO check all
    } else {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        if (send_len != 0 && send != nullptr) {
            //tx with optional rx (after tx)
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write(cmd, (uint8_t*)send, send_len, true);
        }
        if (recv_len != 0 && recv != nullptr) {
            //rx only or rx after tx
            //rx separated from tx by (re)start
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (_address << 1) | I2C_MASTER_READ, true);
            i2c_master_read(cmd, (uint8_t *)recv, recv_len, I2C_MASTER_LAST_NACK);
        }
        i2c_master_stop(cmd);

        uint32_t timeout_ms = 1 + 16L * (send_len + recv_len) * 1000 / bus.bus_clock;
        timeout_ms = MAX(timeout_ms, _timeout_ms);

#if HAL_I2C_CLEAR_ON_TIMEOUT
        bool bus_cleared = false;  // Track if we already attempted bus recovery
#endif

        for (int i = 0; !result && i < _retries; i++) {
            esp_err_t ret = i2c_master_cmd_begin(bus.port, cmd, pdMS_TO_TICKS(timeout_ms));
            result = (ret == ESP_OK);

            if (!result) {
                i2c_reset_tx_fifo(bus.port);
                i2c_reset_rx_fifo(bus.port);

#if HAL_I2C_CLEAR_ON_TIMEOUT
                // OPTIMIZATION: Check SDA on FIRST failure, not after multiple retries
                // This saves time by doing bus recovery immediately if bus is stuck
                if (!bus_cleared && bus.read_sda() == 0) {
                    // SDA is stuck low - device is holding the line
                    // Perform bus recovery immediately (SAFE method - GPIO toggle only)
                    bus.clear_bus();
                    bus_cleared = true;  // Only do this once per transfer

                    // Small delay after bus recovery to let device settle
                    hal.scheduler->delay_microseconds(100);

                    // Continue retry loop - bus recovery should have fixed it
                }
#endif

#if HAL_I2C_INTERNAL_CHECKS
                // Log detailed error on final retry
                if (i == _retries - 1) {
                    const char* err_str = "UNKNOWN";
                    switch (ret) {
                        case ESP_ERR_INVALID_ARG: err_str = "INVALID_ARG"; break;
                        case ESP_FAIL: err_str = "FAIL"; break;
                        case ESP_ERR_INVALID_STATE: err_str = "INVALID_STATE"; break;
                        case ESP_ERR_TIMEOUT: err_str = "TIMEOUT"; break;
                    }
                    printf("I2C: Transfer failed (addr=0x%02x, err=%s, retries=%d, bus_recovery=%s)\n",
                           _address, err_str, _retries, bus_cleared ? "yes" : "no");
                }
#endif
            }
        }

        i2c_cmd_link_delete(cmd);
    }

    return result;
}

/*
  Read multiple  registers in one transaction
  This is an important optimization for high-speed sensors (e.g., IMUs)
  Based on ChibiOS implementation
*/
bool I2CDevice::read_registers_multiple(uint8_t first_reg, uint8_t *recv,
                                        uint32_t recv_len, uint8_t times)
{
    if (times == 0) {
        return false;
    }

    if (!bus.semaphore.check_owner()) {
        printf("I2C: Semaphore not owned (addr=0x%02x)\n", _address);
        return false;
    }

    // For ESP32, we'll do multiple single reads for now
    // Hardware I2C repeated start support exists but needs careful handling
    const uint8_t reg = first_reg;
    uint32_t bytes_per_read = recv_len;

    for (uint8_t i = 0; i < times; i++) {
        if (!transfer(&reg, 1, recv + (i * bytes_per_read), bytes_per_read)) {
            return false;
        }
    }

    return true;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}


/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::I2CDevice *
I2CDeviceManager::get_device_ptr(uint8_t bus, uint8_t address,
                                 uint32_t bus_clock,
                                 bool use_smbus,
                                 uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(i2c_bus_desc)) {
        return nullptr;
    }
    return NEW_NOTHROW I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms);
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return ((1U << ARRAY_SIZE(i2c_bus_desc)) - 1);
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    uint32_t result = 0;
    for (size_t i = 0; i < ARRAY_SIZE(i2c_bus_desc); i++) {
        if (i2c_bus_desc[i].internal) {
            result |= (1u << i);
        }
    }
    return result;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return get_bus_mask() & ~get_bus_mask_internal();
}
