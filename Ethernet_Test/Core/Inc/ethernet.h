/*
 * ethernet.h
 *
 *  Created on: Jan 19, 2026
 *      Author: Abhineet
 */

#ifndef INC_ETHERNET_H_
#define INC_ETHERNET_H_

#include <stdint.h>

typedef struct {
	uint32_t TDES0;
	uint32_t TDES1;
	uint32_t TDES2;
	uint32_t TDES3;
} ETH_TxDMADescTypeDef;

typedef struct {
	uint32_t RDES0;
	uint32_t RDES1;
	uint32_t RDES2;
	uint32_t RDES3;
} ETH_RxDMADescTypeDef;

void ETH_init();
void ETH_start();

void ETH_config_SMI(uint8_t phy_addr);
void ETH_write_PHY(uint8_t reg_addr, uint16_t write_data, uint8_t check_busy);
uint16_t ETH_read_PHY(uint8_t reg_addr);

void ETH_InterruptRequestHandler();

#endif /* INC_ETHERNET_H_ */
