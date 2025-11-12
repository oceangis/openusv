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

#include "Flash.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

using namespace ESP32;

Flash::Flash()
    : partition(nullptr)
    , num_pages(0)
    , base_addr(0)
    , _keep_unlocked(false)
{
    // Find the OTA partition for firmware updates
    // Try to find ota_0 partition first
    partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);

    if (!partition) {
        // Fall back to factory partition if OTA not available
        partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    }

    if (partition) {
        base_addr = partition->address;
        num_pages = partition->size / FLASH_SECTOR_SIZE;
        // Note: Cannot use hal.console here as it's not yet initialized
        // during static construction phase
    }
    // If partition is not found, num_pages will remain 0, which will cause
    // all flash operations to fail safely
}

uint32_t Flash::getpageaddr(uint32_t page)
{
    if (page >= num_pages) {
        return 0;
    }
    return base_addr + (page * FLASH_SECTOR_SIZE);
}

uint32_t Flash::getpagesize(uint32_t page)
{
    if (page >= num_pages) {
        return 0;
    }
    return FLASH_SECTOR_SIZE;
}

uint32_t Flash::getnumpages(void)
{
    return num_pages;
}

bool Flash::erasepage(uint32_t page)
{
    if (!partition || page >= num_pages) {
        return false;
    }

    WITH_SEMAPHORE(sem);

    uint32_t offset = page * FLASH_SECTOR_SIZE;
    esp_err_t err = esp_partition_erase_range(partition, offset, FLASH_SECTOR_SIZE);

    if (err != ESP_OK) {
        hal.console->printf("ESP32::Flash: Erase failed at page %u, error: %d\n",
                          (unsigned)page, err);
        return false;
    }

    return true;
}

bool Flash::write(uint32_t addr, const void *buf, uint32_t count)
{
    if (!partition || !buf || count == 0) {
        return false;
    }

    // Check if address is within partition bounds
    if (addr < base_addr || (addr + count) > (base_addr + partition->size)) {
        hal.console->printf("ESP32::Flash: Write out of bounds: addr=0x%08x, count=%u\n",
                          (unsigned)addr, (unsigned)count);
        return false;
    }

    WITH_SEMAPHORE(sem);

    // Calculate offset within partition
    uint32_t offset = addr - base_addr;

    // ESP32 flash write must be 4-byte aligned
    if (offset % 4 != 0) {
        hal.console->printf("ESP32::Flash: Write address not 4-byte aligned: 0x%08x\n",
                          (unsigned)addr);
        return false;
    }

    esp_err_t err = esp_partition_write(partition, offset, buf, count);

    if (err != ESP_OK) {
        hal.console->printf("ESP32::Flash: Write failed at addr 0x%08x, error: %d\n",
                          (unsigned)addr, err);
        return false;
    }

    return true;
}

void Flash::keep_unlocked(bool set)
{
    // ESP32 flash doesn't require explicit lock/unlock
    // This is a no-op but we track the state for compatibility
    _keep_unlocked = set;
}

bool Flash::ispageerased(uint32_t page)
{
    if (!partition || page >= num_pages) {
        return false;
    }

    WITH_SEMAPHORE(sem);

    uint32_t offset = page * FLASH_SECTOR_SIZE;

    // Allocate buffer to check if page is erased (all 0xFF)
    uint8_t *check_buf = (uint8_t *)malloc(FLASH_SECTOR_SIZE);
    if (!check_buf) {
        return false;
    }

    esp_err_t err = esp_partition_read(partition, offset, check_buf, FLASH_SECTOR_SIZE);
    if (err != ESP_OK) {
        free(check_buf);
        return false;
    }

    // Check if all bytes are 0xFF (erased state)
    bool is_erased = true;
    for (uint32_t i = 0; i < FLASH_SECTOR_SIZE; i++) {
        if (check_buf[i] != 0xFF) {
            is_erased = false;
            break;
        }
    }

    free(check_buf);
    return is_erased;
}
