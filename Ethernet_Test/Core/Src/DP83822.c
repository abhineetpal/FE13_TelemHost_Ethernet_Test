/*
 * DP83822.c
 *
 *  Created on: Jan 28, 2026
 *      Author: Abhineet
 */


#include "stm32f7xx_hal.h"

#include "DP83822.h"
#include "ethernet.h"

void DP83822_init() {
	HAL_Delay(1); // wait one MDC cycle before first SMI transaction (1 ms is way more than one cycle)

	ETH_config_SMI(0x01); // default PHY address
	ETH_write_PHY(0x00, 0x6100, 0); // enable MII loopback, set speed to 100 Mbps, disable auto-negotiation
	ETH_write_PHY(0x16, 0x0120, 1); // enable transmitting while in MII loopback
}
