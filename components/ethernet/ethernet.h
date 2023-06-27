/////////////////////////////////////////////////////////////////
//
// Author:      Martin Landsiedel <marland.dev@gmail.com>
// Date:        26.06.2023
// Desc:        Ethernet support for various chips
//
/////////////////////////////////////////////////////////////////


#include <stdint.h>


/* Get used chip and pinout from the used board.
   To support externally added ethernet chips, add an
   entry with the pinout and functions below.
*/
#ifdef ARDUINO_WT32_ETH01

  #define ETHERNET_ENABLED  1
  #define ETH_CHIP  LAN8720

//#elifdef XXXXX

#endif


/* Function prototypes */
// General
void ethernet_init(void);
esp_err_t eth_pkt_wifi2eth(void* buffer, uint16_t len, void* eb);
// Chip-specific
esp_eth_handle_t lan8720_eth_init(void);


/* Global variables */
extern bool ap_connect;       // Whether the Wifi STA is connected
