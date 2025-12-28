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
//#define HAL_ESP32_SDCARD 1


#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#include "SdCard.h"

#include "esp_vfs_fat.h"
#include "esp_ota_ops.h"

#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include <sys/stat.h>
#include <sys/unistd.h>
#include <sys/types.h>
#include "SPIDevice.h"

#ifdef HAL_ESP32_SDCARD

#if CONFIG_IDF_TARGET_ESP32S2 ||CONFIG_IDF_TARGET_ESP32C3
#define SPI_DMA_CHAN    host.slot
#else
#define SPI_DMA_CHAN    1
#endif

sdmmc_card_t* card = nullptr;

const size_t buffer_size = 8*1024;
const char* fw_name = "/SDCARD/APM/ardupilot.bin";

static bool sdcard_running;
static HAL_Semaphore sem;

void update_fw()
{
    FILE *f = fopen(fw_name, "r");
    void *buffer = calloc(1, buffer_size);
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    size_t nread = 0;
    if (f == nullptr || buffer == nullptr || update_partition == nullptr) {
        goto done;
    }
    printf("updating firmware...\n");
    if (esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle) != ESP_OK) {
        goto done;
    }
    do {
        nread = fread(buffer, 1, buffer_size, f);
        if (nread > 0) {
            if (esp_ota_write(update_handle, buffer, nread) != ESP_OK) {
                goto done;
            }
        }
    } while (nread > 0);
done:
    if (update_handle != 0) {
        if (esp_ota_end(update_handle) == ESP_OK &&
            esp_ota_set_boot_partition(update_partition) == ESP_OK) {
            printf("firmware updated\n");
        }
    }
    if (f != nullptr) {
        fclose(f);
    }
    if (buffer != nullptr) {
        free(buffer);
    }
    unlink(fw_name);
}

#ifdef HAL_ESP32_SDMMC

void mount_sdcard_mmc()
{
    printf("Mounting sd \n");
    WITH_SEMAPHORE(sem);

    /*
    ESP32-S3 SDMMC GPIO Configuration (from schematic):
    - CLK  = GPIO6
    - CMD  = GPIO7
    - DAT0 = GPIO5
    - DAT1 = GPIO4
    - DAT2 = GPIO16
    - DAT3 = GPIO15

    Using 4-bit mode for maximum performance.
    All lines have external 10k pull-ups on PCB.
    */

    static const char *TAG = "SD...";
    ESP_LOGI(TAG, "Initializing SD card as SDMMC (ESP32-S3)");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;  // 20MHz for stability

    // Configure slot with custom GPIO pins for ESP32-S3
    sdmmc_slot_config_t slot_config = {
        .clk = GPIO_NUM_6,
        .cmd = GPIO_NUM_7,
        .d0 = GPIO_NUM_5,
        .d1 = GPIO_NUM_4,
        .d2 = GPIO_NUM_16,
        .d3 = GPIO_NUM_15,
        .d4 = GPIO_NUM_NC,  // Not used in 4-bit mode
        .d5 = GPIO_NUM_NC,
        .d6 = GPIO_NUM_NC,
        .d7 = GPIO_NUM_NC,
        .cd = SDMMC_SLOT_NO_CD,
        .wp = SDMMC_SLOT_NO_WP,
        .width = 4,  // 4-bit mode
        .flags = 0,
    };

    // Enable internal pull-ups (external 10k pull-ups also present on PCB)
    gpio_set_pull_mode(GPIO_NUM_7, GPIO_PULLUP_ONLY);   // CMD
    gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);   // DAT0
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);   // DAT1
    gpio_set_pull_mode(GPIO_NUM_16, GPIO_PULLUP_ONLY);  // DAT2
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);  // DAT3

    // Mount configuration
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = 16 * 1024,
        .disk_status_check_enable = false,
        .use_one_fat = false
    };

    // Mount SD card with FAT filesystem
    sdmmc_card_t* card = NULL;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/SDCARD", &host, &slot_config, &mount_config, &card);

    if (ret == ESP_OK) {
        // Success - print card info
        ESP_LOGI(TAG, "SD Card mounted successfully!");
        ESP_LOGI(TAG, "  Name: %s", card->cid.name);
        ESP_LOGI(TAG, "  Size: %lluMB", ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));

        mkdir("/SDCARD/APM", 0777);
        mkdir("/SDCARD/APM/LOGS", 0777);
        printf("sdcard is mounted (4-bit SDMMC mode)\n");
        sdcard_running = true;
    } else {
        // Failed - detailed error handling
        ESP_LOGE(TAG, "SD card mount failed: %s (0x%x)", esp_err_to_name(ret), ret);

        switch(ret) {
            case ESP_ERR_INVALID_STATE:
                ESP_LOGE(TAG, "Cause: Invalid state - filesystem may not be FAT32");
                ESP_LOGE(TAG, "Solution: Format SD card as FAT32 (not exFAT/NTFS)");
                break;
            case ESP_FAIL:
                ESP_LOGE(TAG, "Cause: No valid FAT partition found");
                ESP_LOGE(TAG, "Solution: Format SD card as FAT32 with MBR partition");
                break;
            case ESP_ERR_TIMEOUT:
                ESP_LOGE(TAG, "Cause: Card communication timeout");
                ESP_LOGE(TAG, "Solution: Check card insertion and GPIO connections");
                break;
            case ESP_ERR_INVALID_RESPONSE:
                ESP_LOGE(TAG, "Cause: Card returned invalid response");
                ESP_LOGE(TAG, "Solution: Try a different SD card");
                break;
            case ESP_ERR_NO_MEM:
                ESP_LOGE(TAG, "Cause: Not enough memory");
                break;
            default:
                ESP_LOGE(TAG, "Check: Card inserted? FAT32 format? Card damaged?");
                break;
        }

        printf("sdcard is not mounted\n");
        sdcard_running = false;
    }
}

void mount_sdcard()
{
    mount_sdcard_mmc();
}

#endif // emd mmc


#ifdef HAL_ESP32_SDSPI
ESP32::SPIBusDesc bus_ = HAL_ESP32_SDSPI;

void mount_sdcard_spi()
{

    esp_err_t ret;
    printf("Mounting sd \n");
    WITH_SEMAPHORE(sem);

    //  In SPI mode, pins can be customized...

    // and 'SPI bus sharing with SDSPI has been added in 067f3d2 — please see the new sdspi_host_init_device, sdspi_host_remove_device functions'. https://github.com/espressif/esp-idf/issues/1597 buzz..: thats in idf circa 4.3-dev tho.

    // readme shows esp32 pinouts and pullups needed for different modes and 1 vs 4 line gotchass...
    //https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/README.md
    //https://github.com/espressif/esp-idf/blob/master/examples/storage/sd_card/main/sd_card_example_main.c

    static const char *TAG = "SD...";
    ESP_LOGI(TAG, "Initializing SD card as SDSPI");
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    //TODO change to sdspi_host_init_device for spi sharing
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = bus_.mosi,
        .miso_io_num = bus_.miso,
        .sclk_io_num = bus_.sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = bus_.cs;
    slot_config.host_id = (spi_host_device_t)host.slot;

    //host.flags = SDMMC_HOST_FLAG_1BIT | SDMMC_HOST_FLAG_DDR;
    host.max_freq_khz = SDMMC_FREQ_PROBING;
    ret = esp_vfs_fat_sdspi_mount("/SDCARD", &host, &slot_config, &mount_config, &card);

    if (ret == ESP_OK) {
        // Card has been initialized, print its properties

        mkdir("/SDCARD/APM", 0777);
        mkdir("/SDCARD/APM/LOGS", 0777);
        printf("sdcard is mounted\n");
        //update_fw();
        sdcard_running = true;
    } else {
        printf("sdcard is not mounted\n");
        sdcard_running = false;
    }
}
void mount_sdcard()
{
    mount_sdcard_spi();
}

#endif // end spi

bool sdcard_retry(void)
{
    if (!sdcard_running) {
        mount_sdcard();
    }
    return sdcard_running;
}

void unmount_sdcard()
{
    if (card != nullptr) {
        esp_vfs_fat_sdcard_unmount( "/SDCARD", card);
    }
    sdcard_running = false;
}

#else
// empty impl's
void mount_sdcard()
{
    printf("No sdcard setup.\n");
}
void unmount_sdcard()
{
}
bool sdcard_retry(void)
{
    return true;
}
#endif




