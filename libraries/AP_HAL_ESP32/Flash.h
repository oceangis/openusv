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

#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"
#include "Semaphores.h"
#include "esp_partition.h"
#include "esp_flash.h"

class ESP32::Flash : public AP_HAL::Flash {
public:
    Flash();

    // Get the address of a given flash page (sector)
    uint32_t getpageaddr(uint32_t page) override;

    // Get the size of a given flash page (sector)
    uint32_t getpagesize(uint32_t page) override;

    // Get the number of flash pages (sectors)
    uint32_t getnumpages(void) override;

    // Erase a flash page (sector)
    bool erasepage(uint32_t page) override;

    // Write data to flash at given address
    bool write(uint32_t addr, const void *buf, uint32_t count) override;

    // Keep flash unlocked (not applicable for ESP32, but required by interface)
    void keep_unlocked(bool set) override;

    // Check if a page is erased
    bool ispageerased(uint32_t page) override;

private:
    HAL_Semaphore sem;
    const esp_partition_t *partition;

    // Flash configuration for ESP32-S3
    static constexpr uint32_t FLASH_SECTOR_SIZE = 4096;  // 4KB sectors
    static constexpr uint32_t FLASH_BASE_ADDR = 0x0;

    // Calculate total number of sectors based on partition size
    uint32_t num_pages;
    uint32_t base_addr;

    bool _keep_unlocked;
};
