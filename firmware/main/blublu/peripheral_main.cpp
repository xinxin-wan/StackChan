/*
 * BluBlu peripheral mode entry.
 *
 * StackChan works as a peripheral of the BluBlu agent on aibox:
 * camera (snapshot / MJPEG stream), head servos, avatar emotion and mouth.
 * No xiaozhi Application, no audio, no OTA, no cloud.
 */
#include "peripheral_main.h"
#include "camera_pipeline.h"
#include "http_api.h"
#include "robot_control.h"
#include <sdkconfig.h>
#if CONFIG_STACKCHAN_BLUBLU_USB_LINK
#include "usb_link.h"
#endif
#include <hal/hal.h>
#include <stackchan/stackchan.h>
#include <board.h>
#include <display.h>
#include <wifi_manager.h>
#include <mdns.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string>

namespace blublu {

static const char* TAG = "BluBlu";

static void update_task(void*)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(20));
        LvglLockGuard lock;
        robot::tick();
        GetStackChan().update();
    }
}

#if CONFIG_STACKCHAN_BLUBLU_WIFI
static void start_mdns()
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return;
    }
    mdns_hostname_set(CONFIG_STACKCHAN_BLUBLU_HOSTNAME);
    mdns_instance_name_set("StackChan BluBlu Peripheral");
    mdns_txt_item_t txt[] = {
        {"api", "/api/v1"},
        {"device", "stackchan-k151"},
    };
    mdns_service_add(nullptr, "_http", "_tcp", CONFIG_STACKCHAN_BLUBLU_HTTP_PORT, txt, 2);
    mdns_service_add(nullptr, "_stackchan", "_tcp", CONFIG_STACKCHAN_BLUBLU_HTTP_PORT, txt, 2);
    ESP_LOGI(TAG, "mDNS: %s.local", CONFIG_STACKCHAN_BLUBLU_HOSTNAME);
}
#endif

void peripheral_main()
{
    ESP_LOGI(TAG, "BluBlu peripheral mode");

    // Avatar UI (without the xiaozhi Application)
    {
        LvglLockGuard lock;
        Board::GetInstance().GetDisplay()->SetupUI();
        GetHAL().bootLogo.reset();
        auto& motion = GetStackChan().motion();
        motion.setAutoAngleSyncEnabled(true);
        motion.setAutoTorqueReleaseEnabled(true);
        // Start from the centre: the servos keep whatever pose they had at power-off
        motion.goHome(300);
    }
    xTaskCreatePinnedToCore(update_task, "bb_update", 4096, nullptr, 3, nullptr, 1);

    if (!camera::start()) {
        ESP_LOGE(TAG, "camera pipeline not started, snapshot/stream will return 503");
    }

    std::string status;

#if CONFIG_STACKCHAN_BLUBLU_USB_LINK
    // USB link: framed protocol on USB-Serial-JTAG, no network interface on the host
    if (usb_link::start()) {
        status = "USB ready";
    } else {
        status = "USB link failed";
    }
    robot::show_text(status, 0);
#endif

#if CONFIG_STACKCHAN_BLUBLU_WIFI
    robot::show_text("WiFi connecting...", 0);
    GetHAL().startNetwork([](std::string_view msg) { robot::show_text(std::string(msg), 0); });
    Board::GetInstance().SetPowerSaveLevel(PowerSaveLevel::PERFORMANCE);

    const std::string ip = WifiManager::GetInstance().GetIpAddress();
    ESP_LOGI(TAG, "network connected, ip %s", ip.c_str());

    start_mdns();
    http::start(CONFIG_STACKCHAN_BLUBLU_HTTP_PORT);
    status += (status.empty() ? "" : "\n") + std::string("IP ") + ip;
#endif

    robot::show_text(status, 15000);

    while (true) {
        GetHAL().delay(1000);
        GetHAL().updateHeapStatusLog();
    }
}

}  // namespace blublu
