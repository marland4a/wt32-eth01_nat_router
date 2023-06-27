/////////////////////////////////////////////////////////////////
//
// Author:      Martin Landsiedel <marland.dev@gmail.com>
// Date:        26.06.2023
// Desc:        Ethernet for WT32-ETH01 (LAN8720 chip)
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
// Pin configurations
#define ETH_CONFIG_MAC_MDC_GPIO         23                // IOs where the Ethernet module is connected to
#define ETH_CONFIG_MAC_MDIO_GPIO        18
#define ETH_CONFIG_MAC_CLK_GPIO         0                 // IO used to output clock for RMII interface
#define ETH_CONFIG_PHY_RESET_GPIO       5                 // IO used to reset ETH module
#define ETH_POWER_GPIO                  16                // IO which enables power to the LAN8720 module


/*** Global Variables =============================== ***/


/*** Prototypes ===================================== ***/
esp_eth_handle_t lan8720_eth_init(void);


/*** Init ethernet interface ======================== ***/
// See https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/network/esp_eth.html
esp_eth_handle_t lan8720_eth_init(void) {
  // Enable power to LAN8720 chip / osc
  gpio_config_t power_config = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = (1<<ETH_POWER_GPIO) | (1<<ETH_CONFIG_PHY_RESET_GPIO),
    .intr_type = GPIO_INTR_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .pull_up_en = GPIO_PULLUP_DISABLE
  };

  // Power reset
  gpio_config(&power_config);                             // Set pin to output
  gpio_set_level(ETH_POWER_GPIO, 1U);                     // Set level high after timeout
  

  // Internal EMAC instance
  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  mac_config.smi_mdc_gpio_num = ETH_CONFIG_MAC_MDC_GPIO;  // Set custom RMII GPIOs
  mac_config.smi_mdio_gpio_num = ETH_CONFIG_MAC_MDIO_GPIO;
  //mac_config.clock_config.rmii.clock_gpio = 0;            // Enable clock output on GPIO0: https://esp32.com/viewtopic.php?t=26745
  //mac_config.clock_config.rmii.clock_mode = EMAC_CLK_OUT;
  esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&mac_config);// Create MAC instance

  // External PHY instance (LAN8720)
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = ESP_ETH_PHY_ADDR_AUTO;            // Automatically find PHY address
  phy_config.reset_gpio_num = ETH_CONFIG_PHY_RESET_GPIO;  // Set custom PHY reset pin
  //esp_eth_phy_t *phy = esp_eth_phy_new_lan8720(&phy_config);// Create PHY instance for LAN8720 Module (used by this board)
  esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);

  if(phy == NULL) {
    ESP_ERROR_CHECK(ESP_FAIL);
  }

  // Install ethernet driver
  esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
  esp_eth_handle_t eth_handle = NULL;                     // We will get handle after driver installed
  //ESP_ERROR_CHECK(
    esp_err_t ret = esp_eth_driver_install(&config, &eth_handle); // Install driver
  //);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Ethernet driver install failed with %s", esp_err_to_name(ret));
    abort();
  }

  // Create event loop
  //esp_event_loop_create_default();                        // Create a default event loop that running in background
                                                            //  -> Done in main
  /*ESP_ERROR_CHECK(
    esp_event_handler_register(ETH_EVENT,                 // Register Ethernet event handler
          ESP_EVENT_ANY_ID, &eth_event_handler, NULL)     //  to deal with user specific stuffs when event like link up/down happened
  );*/                                                    //  -> executed by caller

  
  /* Do not use TCP/IP here, but use Layer 2 forwarding

  // Initialize TCP/IP network interface
  //esp_netif_init();                                       // Initialize TCP/IP network interface (should be called only once in application)
                                                            //   -> Moved to app_main()
  esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();       // apply default network interface configuration for Ethernet
  eth_netif = esp_netif_new(&cfg);                        // create network interface for Ethernet driver (global variable)

  // Attach Ethernet driver to TCP/IP stack
  ESP_ERROR_CHECK(
    esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)) // attach Ethernet driver to TCP/IP stack
  );
  //esp_event_handler_register(IP_EVENT,
  //      IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL);// Register Got-IP Event handler

  
  // Enable Flow control (request to pause transmission if peer sends too fast)
  bool flow_ctrl_enable = true;
  esp_eth_ioctl(eth_handle, ETH_CMD_S_FLOW_CTRL, &flow_ctrl_enable);


  // Set interface hostname
  //esp_netif_set_hostname(eth_netif, NAME);
  // Set interface IP
  //lan8720_eth_set_ip(ETH_IP_ADDR, ETH_IP_NETMASK, ETH_IP_GATEWAY);

  // Set eth ip address and start DHCP server
  uint32_t my_ap_ip = ipaddr_addr(ap_ip);
  //lan8720_eth_set_ip((my_ap_ip & 0x00FFFFFF) | 0xFE000000, 0x00FFFFFF, my_ap_ip);
  lan8720_eth_set_ip(my_ap_ip, 0x00FFFFFF, my_ap_ip);
  /'esp_netif_ip_info_t ipInfo_eth;
  ipInfo_eth.ip.addr = (my_ap_ip & 0xFFFFFF00) | 0xFE;   // Set eth ip to xxx.xxx.xxx.254
  ipInfo_eth.gw.addr = my_ap_ip;
  IP4_ADDR(&ipInfo_eth.netmask, 255,255,255,0);
  esp_netif_dhcps_stop(eth_netif); // stop before setting ip
  esp_netif_set_ip_info(eth_netif, &ipInfo_eth);
  '/
  esp_netif_dhcps_start(eth_netif);
  
  /'
  dhcps_offer_t dhcps_dns_value = OFFER_DNS;
  dhcps_set_option_info(6, &dhcps_dns_value, sizeof(dhcps_dns_value));
  // Set custom dns server address for dhcp server
  ip_addr_t dnsserver;
  dnsserver.u_addr.ip4.addr = ipaddr_addr("8.8.8.8");
  dnsserver.type = IPADDR_TYPE_V4;
  dhcps_dns_setserver(&dnsserver);
  '/

  esp_event_handler_register(IP_EVENT,
        IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL);// Register Got-IP Event handler

  Unused Layer 3 functionality */

  /*// Start ethernet driver state machine -> Executed by caller
  ESP_ERROR_CHECK(
    esp_eth_start(eth_handle)
  );
  */
  return eth_handle;
}
