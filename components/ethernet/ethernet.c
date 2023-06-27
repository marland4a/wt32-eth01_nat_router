/////////////////////////////////////////////////////////////////
//
// Author:      Martin Landsiedel <marland.dev@gmail.com>
// Date:        26.06.2023
// Desc:        General ethernet functions.
//              Inspired by the esp-idf eth2ap example
//
/////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_phy_init.h"
#include "esp_eth_phy.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "hal/gpio_types.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "ethernet.h"


/*** Global constants =============================== ***/
static const char *TAG = "EthernetSetup";                 // Tag displayed in serial logs
#define FLOW_CONTROL_QUEUE_TIMEOUT_MS     (100)
#define FLOW_CONTROL_QUEUE_LENGTH         (40)
#define FLOW_CONTROL_WIFI_SEND_TIMEOUT_MS (100)


/*** Global Variables =============================== ***/
static esp_eth_handle_t eth_handle = NULL;                // Handle to eth config
static bool eth_connected = false;                        // Whether the ethernet interface is up or down
static QueueHandle_t flow_control_queue = NULL;


/*** Prototypes ===================================== ***/
typedef struct {
    void *packet;
    uint16_t length;
} flow_control_msg_t;

void ethernet_init(void);
static esp_err_t init_flow_control(void);
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data);
esp_err_t eth_pkt_wifi2eth(void* buffer, uint16_t len, void* eb);
static esp_err_t eth_pkt_eth2wifi(esp_eth_handle_t eth_handle, uint8_t* buffer,
                              uint32_t len, void* priv);
static void eth2wifi_flow_control_task(void* args);



/*** Initialize ethernet ============================ ***/
void ethernet_init(void) {
  // Initialize flow control for layer 2 forwarding before initializing ethernet
  ESP_ERROR_CHECK(init_flow_control());

  // Initialize the specified ethernet chip
  #if ETH_CHIP == LAN8720
    eth_handle = lan8720_eth_init();
  #else
    #error No ethernet setup for the specified chip defined
  #endif

  // General setup for Layer 2 forwarding
  ESP_ERROR_CHECK(esp_eth_update_input_path(eth_handle, eth_pkt_eth2wifi, NULL));
  bool eth_promiscuous = true;
  ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_PROMISCUOUS, &eth_promiscuous));
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL));
  ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}


/*** General Ethernet event handler ================= ***/
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data) {
  uint8_t mac_addr[6] = {0};
  // We can get the ethernet driver handle from event data
  esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

  switch (event_id) {
  case ETHERNET_EVENT_CONNECTED:
    esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
    ESP_LOGI(TAG, "Ethernet Link Up");
    ESP_LOGI(TAG, "Ethernet HW Addr: %02x:%02x:%02x:%02x:%02x:%02x",
                mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    eth_connected = true;

    /* Set custom static ip address */
    //lan8720_eth_set_ip(ETH_IP_ADDR, ETH_IP_NETMASK, ETH_IP_GATEWAY);
    //esp_netif_dhcpc_start(eth_netif);

    break;
  case ETHERNET_EVENT_DISCONNECTED:
    ESP_LOGI(TAG, "Ethernet Link Down");
    eth_connected = false;
    break;
  case ETHERNET_EVENT_START:
    ESP_LOGI(TAG, "Ethernet Started");
    break;
  case ETHERNET_EVENT_STOP:
    ESP_LOGI(TAG, "Ethernet Stopped");
    break;
  default:
    break;
  }
}


/*** Setup for Layer 2 forwarding =================== ***/
static esp_err_t init_flow_control(void) {
  flow_control_queue = xQueueCreate(FLOW_CONTROL_QUEUE_LENGTH, sizeof(flow_control_msg_t));
  if (!flow_control_queue) {
      ESP_LOGE(TAG, "create flow control queue failed");
      return ESP_FAIL;
  }
  BaseType_t ret = xTaskCreate(eth2wifi_flow_control_task, "flow_ctl", 2048, NULL, (tskIDLE_PRIORITY + 2), NULL);
  if (ret != pdTRUE) {
      ESP_LOGE(TAG, "create flow control task failed");
      return ESP_FAIL;
  }
  return ESP_OK;
}


/*** Layer 2 forwarding logic ======================= ***/
/* Forward packets from Wi-Fi to Ethernet */
esp_err_t eth_pkt_wifi2eth(void *buffer, uint16_t len, void *eb) {
  if (eth_connected) {
    if (esp_eth_transmit(eth_handle, buffer, len) != ESP_OK) {
      ESP_LOGE(TAG, "Ethernet send packet failed");
    }
  }
  esp_wifi_internal_free_rx_buffer(eb);
  return ESP_OK;
}

/* Forward packets from Ethernet to Wi-Fi */
// Note that, Ethernet works faster than Wi-Fi on ESP32,
// so we need to add an extra queue to balance their speed difference.
static esp_err_t eth_pkt_eth2wifi(esp_eth_handle_t eth_handle, uint8_t *buffer,
                                  uint32_t len, void *priv) {
    esp_err_t ret = ESP_OK;
    flow_control_msg_t msg = {
        .packet = buffer,
        .length = len
    };
    if (xQueueSend(flow_control_queue, &msg, pdMS_TO_TICKS(FLOW_CONTROL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "send flow control message failed or timeout");
        free(buffer);
        ret = ESP_FAIL;
    }
    return ret;
}

/* This task will fetch the packet from the queue, and then send out through Wi-Fi */
// Wi-Fi handles packets slower than Ethernet, we might add some delay between each transmitting.
static void eth2wifi_flow_control_task(void *args) {
    flow_control_msg_t msg;
    int res = 0;
    uint32_t timeout = 0;
    while (1) {
        if (xQueueReceive(flow_control_queue, &msg, pdMS_TO_TICKS(FLOW_CONTROL_QUEUE_TIMEOUT_MS)) == pdTRUE) {
            timeout = 0;
            if (ap_connect && msg.length) {
                do {
                    vTaskDelay(pdMS_TO_TICKS(timeout));
                    timeout += 2;
                    //res = esp_wifi_internal_tx(WIFI_IF_AP, msg.packet, msg.length);
                    res = esp_wifi_internal_tx(WIFI_IF_STA, msg.packet, msg.length);
                } while (res && timeout < FLOW_CONTROL_WIFI_SEND_TIMEOUT_MS);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "WiFi send packet failed: %d", res);
                }
            }
            free(msg.packet);
        }
    }
    vTaskDelete(NULL);
}