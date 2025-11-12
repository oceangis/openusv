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
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include "Storage.h"

#define STORAGEDEBUG 1

using namespace ESP32;

extern const AP_HAL::HAL& hal;

void Storage::_storage_open(void)
{
    if (_initialised) {
        return;
    }
#ifdef STORAGEDEBUG
    printf("%s:%d _storage_open \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    _dirty_mask.clearall();

    // Initialize _last_empty_ms to current time to make healthy() work
    _last_empty_ms = AP_HAL::millis();

    p = esp_partition_find_first((esp_partition_type_t)0x45, ESP_PARTITION_SUBTYPE_ANY, nullptr);

    // Check if partition was found
    if (p == nullptr) {
        printf("ERROR: Storage partition (type 0x45) not found!\n");
        printf("  This indicates the partition table was not properly flashed.\n");
        printf("  Please run: idf.py -p COM_PORT partition-table-flash\n");
        printf("  Falling back to empty storage (calibration data will NOT persist)\n");
        _use_empty_storage = true;
        _initialised = true;
        return;
    }


    printf("Storage partition found: addr=0x%08x, size=%u bytes\n",
           p->address, p->size);

    // load from storage backend
    _flash_load();
    _initialised = true;
}

/*
  mark some lines as dirty. Note that there is no attempt to avoid
  the race condition between this code and the _timer_tick() code
  below, which both update _dirty_mask. If we lose the race then the
  result is that a line is written more than once, but it won't result
  in a line not being written.
*/
void Storage::_mark_dirty(uint16_t loc, uint16_t length)
{
#ifdef STORAGEDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    uint16_t end = loc + length;
    for (uint16_t line=loc>>STORAGE_LINE_SHIFT;
         line <= end>>STORAGE_LINE_SHIFT;
         line++) {
        _dirty_mask.set(line);
    }
}

void Storage::read_block(void *dst, uint16_t loc, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
#ifdef STORAGEDEBUG
        printf("%s:%d read_block failed \n", __PRETTY_FUNCTION__, __LINE__);
#endif
        return;
    }
    _storage_open();
    memcpy(dst, &_buffer[loc], n);
}

void Storage::write_block(uint16_t loc, const void *src, size_t n)
{
    if (loc >= sizeof(_buffer)-(n-1)) {
#ifdef STORAGEDEBUG
        printf("%s:%d write_block failed \n", __PRETTY_FUNCTION__, __LINE__);
#endif
        return;
    }
    if (memcmp(src, &_buffer[loc], n) != 0) {
        _storage_open();
        memcpy(&_buffer[loc], src, n);
        _mark_dirty(loc, n);
    }
}

void Storage::_timer_tick(void)
{
    if (!_initialised) {
        return;
    }
    if (_dirty_mask.empty()) {
        _last_empty_ms = AP_HAL::millis();
        return;
    }

    // write out the first dirty line. We don't write more
    // than one to keep the latency of this call to a minimum
    uint16_t i;
    for (i=0; i<STORAGE_NUM_LINES; i++) {
        if (_dirty_mask.get(i)) {
            break;
        }
    }
    if (i == STORAGE_NUM_LINES) {
        // this shouldn't be possible
        return;
    }

    // save to storage backend
    _flash_write(i);
}

/*
  load all data from flash
 */
void Storage::_flash_load(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_use_empty_storage) {
        // Already using empty storage, no need to init flash
        return;
    }

    if (!_flash.init()) {
        // Flash init failed, fallback to empty storage
        printf("WARNING: Unable to init flash storage, falling back to empty storage\n");
        _use_empty_storage = true;
        return;
    }
}

/*
  write one storage line. This also updates _dirty_mask.
  Enhanced with write verification
*/
void Storage::_flash_write(uint16_t line)
{
#ifdef STORAGEDEBUG
    printf("%s:%d \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    if (_flash.write(line*STORAGE_LINE_SIZE, STORAGE_LINE_SIZE)) {
        _write_count++;

        // Verify write if not using empty storage
        if (!_use_empty_storage) {
            uint8_t verify_buffer[STORAGE_LINE_SIZE];
            uint32_t offset = line * STORAGE_LINE_SIZE;

            if (_flash_read_data(0, offset, verify_buffer, STORAGE_LINE_SIZE)) {
                // Compare written data with what we intended to write
                if (memcmp(&_buffer[offset], verify_buffer, STORAGE_LINE_SIZE) == 0) {
                    // Write verified successfully
                    _dirty_mask.clear(line);
                } else {
                    // Write verification failed - keep line dirty for retry
                    _verify_fail_count++;
#ifdef STORAGEDEBUG
                    printf("Storage write verification failed at line %u\n", line);
#endif
                }
            } else {
                // Read back failed - keep line dirty
                _verify_fail_count++;
            }
        } else {
            // Empty storage - just mark clean
            _dirty_mask.clear(line);
        }
    }
}

/*
  callback to write data to flash
 */
bool Storage::_flash_write_data(uint8_t sector, uint32_t offset, const uint8_t *data, uint16_t length)
{
    if (p == nullptr || _use_empty_storage) {
        // Using empty storage, writes are ignored but return success
        return true;
    }

#ifdef STORAGEDEBUG
    printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    size_t address = sector * STORAGE_SECTOR_SIZE + offset;
    bool ret = esp_partition_write(p, address, data, length) == ESP_OK;
    if (!ret && _flash_erase_ok()) {
        // we are getting flash write errors while disarmed. Try
        // re-writing all of flash
        uint32_t now = AP_HAL::millis();
        if (now - _last_re_init_ms > 5000) {
            _last_re_init_ms = now;
            bool ok = _flash.re_initialise();
            DEV_PRINTF("Storage: failed at %u:%u for %u - re-init %u\n",
                                (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)ok);
#ifdef STORAGEDEBUG
            printf("Storage: failed at %u:%u for %u - re-init %u\n",
                   (unsigned)sector, (unsigned)offset, (unsigned)length, (unsigned)ok);
#endif
        }
    }
    return ret;
}

/*
  callback to read data from flash
 */
bool Storage::_flash_read_data(uint8_t sector, uint32_t offset, uint8_t *data, uint16_t length)
{
    if (p == nullptr || _use_empty_storage) {
        // Using empty storage, return zeros
        memset(data, 0, length);
        return true;
    }

    size_t address = sector * STORAGE_SECTOR_SIZE + offset;
#ifdef STORAGEDEBUG
    printf("%s:%d  -> sec:%u off:%d len:%d addr:%d\n", __PRETTY_FUNCTION__, __LINE__,sector,offset,length,address);
#endif
    esp_err_t ret = esp_partition_read(p, address, data, length);
    return (ret == ESP_OK);
}

/*
  callback to erase flash sector
 */
bool Storage::_flash_erase_sector(uint8_t sector)
{
    if (p == nullptr || _use_empty_storage) {
        // Using empty storage, erase is a no-op but returns success
        return true;
    }

#ifdef STORAGEDEBUG
    printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    size_t address = sector * STORAGE_SECTOR_SIZE;
    return esp_partition_erase_range(p, address, STORAGE_SECTOR_SIZE) == ESP_OK;
}

/*
  callback to check if erase is allowed
 */
bool Storage::_flash_erase_ok(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    // only allow erase while disarmed
    return !hal.util->get_soft_armed();
}

/*
  consider storage healthy if we have nothing to write sometime in the
  last 2 seconds
 */
bool Storage::healthy(void)
{
#ifdef STORAGEDEBUG
    printf("%s:%d  \n", __PRETTY_FUNCTION__, __LINE__);
#endif
    return _initialised && AP_HAL::millis() - _last_empty_ms < 2000;
}

/*
  get storage size and ptr
 */
bool Storage::get_storage_ptr(void *&ptr, size_t &size)
{
    if (!_initialised) {
        return false;
    }
    ptr = _buffer;
    size = sizeof(_buffer);
    return true;
}

/*
  Calculate CRC32 checksum for data integrity verification
  Based on ChibiOS implementation approach
 */
uint32_t Storage::calculate_checksum(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

/*
  Verify storage integrity by checking data consistency
  Returns true if storage data is valid
 */
bool Storage::verify_storage_integrity(void)
{
    if (!_initialised || _use_empty_storage) {
        return true;  // Empty storage is always "valid"
    }

    // Calculate checksum of current buffer
    uint32_t current_checksum = calculate_checksum(_buffer, STORAGE_SIZE);

    // Read back from flash and verify
    uint8_t verify_buffer[STORAGE_LINE_SIZE];
    bool all_valid = true;

    for (uint16_t line = 0; line < STORAGE_NUM_LINES; line++) {
        uint32_t offset = line * STORAGE_LINE_SIZE;

        if (_flash_read_data(0, offset, verify_buffer, STORAGE_LINE_SIZE)) {
            // Compare with buffer
            if (memcmp(&_buffer[offset], verify_buffer, STORAGE_LINE_SIZE) != 0) {
                // Mismatch detected - this is expected for dirty lines
                if (!_dirty_mask.get(line)) {
                    // Clean line shouldn't have mismatch
                    all_valid = false;
                    _verify_fail_count++;
#ifdef STORAGEDEBUG
                    printf("Storage verification failed at line %u\n", line);
#endif
                }
            }
        } else {
            all_valid = false;
            _verify_fail_count++;
        }
    }

    if (all_valid) {
        _storage_checksum = current_checksum;
    }

    return all_valid;
}
