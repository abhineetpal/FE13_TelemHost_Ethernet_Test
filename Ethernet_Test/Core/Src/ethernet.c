/*
 * ethernet.c
 *
 *  Created on: Jan 19, 2026
 *      Author: Abhineet
 */


#include "stm32f7xx_hal.h"

#include "ethernet.h"
#include "main.h"

static const uint32_t ETH_BASE_ADDR = 0x40028000;
static const uint32_t ETH_MACCR_ADDR_OFFSET = 0x0000;
static const uint32_t ETH_MACFFR_ADDR_OFFSET = 0x0004;
static const uint32_t ETH_MACMIIAR_ADDR_OFFSET = 0x0010;
static const uint32_t ETH_MACMIIDR_ADDR_OFFSET = 0x0014;
static const uint32_t ETH_MACDBGR_ADDR_OFFSET = 0x0034;
static const uint32_t ETH_DMABMR_ADDR_OFFSET = 0x1000;
static const uint32_t ETH_DMATPDR_ADDR_OFFSET = 0x1004;
static const uint32_t ETH_DMARPDR_ADDR_OFFSET = 0x1008;
static const uint32_t ETH_DMARDLAR_ADDR_OFFSET = 0x100C;
static const uint32_t ETH_DMATDLAR_ADDR_OFFSET = 0x1010;
static const uint32_t ETH_DMASR_ADDR_OFFSET = 0x1014;
static const uint32_t ETH_DMAOMR_ADDR_OFFSET = 0x1018;
static const uint32_t ETH_DMAIER_ADDR_OFFSET = 0x101C;

volatile uint32_t* const ETH_MACCR = (uint32_t*)(ETH_BASE_ADDR + ETH_MACCR_ADDR_OFFSET); // MAC config register
volatile uint32_t* const ETH_MACFFR = (uint32_t*)(ETH_BASE_ADDR + ETH_MACFFR_ADDR_OFFSET); // MAC frame filter register
volatile uint32_t* const ETH_MACMIIAR = (uint32_t*)(ETH_BASE_ADDR + ETH_MACMIIAR_ADDR_OFFSET); // MAC MII address register
volatile uint32_t* const ETH_MACMIIDR = (uint32_t*)(ETH_BASE_ADDR + ETH_MACMIIDR_ADDR_OFFSET); // MAC MII data register
volatile uint32_t* const ETH_MACDBGR = (uint32_t*)(ETH_BASE_ADDR + ETH_MACDBGR_ADDR_OFFSET); // MAC debug register
volatile uint32_t* const ETH_DMABMR = (uint32_t*)(ETH_BASE_ADDR + ETH_DMABMR_ADDR_OFFSET); // DMA bus mode register
volatile uint32_t* const ETH_DMATPDR = (uint32_t*)(ETH_BASE_ADDR + ETH_DMATPDR_ADDR_OFFSET); // DMA transmit poll demand register
volatile uint32_t* const ETH_DMARPDR = (uint32_t*)(ETH_BASE_ADDR + ETH_DMARPDR_ADDR_OFFSET); // DMA receive poll demand register
volatile uint32_t* const ETH_DMARDLAR = (uint32_t*)(ETH_BASE_ADDR + ETH_DMARDLAR_ADDR_OFFSET); // DMA receive descriptor list address register
volatile uint32_t* const ETH_DMATDLAR = (uint32_t*)(ETH_BASE_ADDR + ETH_DMATDLAR_ADDR_OFFSET); // DMA transmit descriptor list address register
volatile uint32_t* const ETH_DMASR = (uint32_t*)(ETH_BASE_ADDR + ETH_DMASR_ADDR_OFFSET); // DMA status register
volatile uint32_t* const ETH_DMAOMR = (uint32_t*)(ETH_BASE_ADDR + ETH_DMAOMR_ADDR_OFFSET); // DMA operation mode register
volatile uint32_t* const ETH_DMAIER = (uint32_t*)(ETH_BASE_ADDR + ETH_DMAIER_ADDR_OFFSET); // DMA interrupt enable register

volatile ETH_RxDMADescTypeDef RxDescriptors[4] __attribute__((section(".RxDescripSection")));
volatile ETH_TxDMADescTypeDef TxDescriptors[4] __attribute__((section(".TxDescripSection")));

extern uint8_t* buf0;
extern uint8_t* buf1;
extern uint8_t* buf2;
extern uint8_t* buf3;
extern uint8_t* buf4;
extern uint8_t* buf5;
extern uint8_t* buf6;
extern uint8_t* buf7;

extern volatile uint8_t* rbuf0;
extern volatile uint8_t* rbuf1;
extern volatile uint8_t* rbuf2;
extern volatile uint8_t* rbuf3;
extern volatile uint8_t* rbuf4;
extern volatile uint8_t* rbuf5;
extern volatile uint8_t* rbuf6;
extern volatile uint8_t* rbuf7;

// Initialize GPIO and clock
void ETH_MspInit() {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_ETH_CLK_ENABLE();

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;

	// Initialize Port A pins
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Initialize Port B pins
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	// Initialize Port C pins
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Initialize Port E pin
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(ETH_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ETH_IRQn);
}

// Creates transmit and receive descriptor lists
void ETH_init_descriptors() {
	TxDescriptors[0].TDES0 = 0x10000000;
	TxDescriptors[0].TDES1 = 0x00080008;
	TxDescriptors[0].TDES2 = (uint32_t)(&buf0);
	TxDescriptors[0].TDES3 = (uint32_t)(&buf1);

	TxDescriptors[1].TDES0 = 0xE0000000;
	TxDescriptors[1].TDES1 = 0x00080008;
	TxDescriptors[1].TDES2 = (uint32_t)(&buf2);
	TxDescriptors[1].TDES3 = (uint32_t)(&buf3);

	TxDescriptors[2].TDES0 = 0x10000000;
	TxDescriptors[2].TDES1 = 0x00080008;
	TxDescriptors[2].TDES2 = (uint32_t)(&buf4);
	TxDescriptors[2].TDES3 = (uint32_t)(&buf5);

	TxDescriptors[3].TDES0 = 0xE0200000;
	TxDescriptors[3].TDES1 = 0x00080008;
	TxDescriptors[3].TDES2 = (uint32_t)(&buf6);
	TxDescriptors[3].TDES3 = (uint32_t)(&buf7);

	// Set OWN bit in first descriptor of frame after setting up other descriptors in frame
	TxDescriptors[0].TDES0 |= 0x80000000;
	TxDescriptors[2].TDES0 |= 0x80000000;

	RxDescriptors[0].RDES0 = 0x80000000;
	RxDescriptors[0].RDES1 = 0x00080000;
	RxDescriptors[0].RDES2 = (uint32_t)(&rbuf0);
	RxDescriptors[0].RDES3 = (uint32_t)(&rbuf1);

	RxDescriptors[1].RDES0 = 0x80000000;
	RxDescriptors[1].RDES1 = 0x00080008;
	RxDescriptors[1].RDES2 = (uint32_t)(&rbuf2);
	RxDescriptors[1].RDES3 = (uint32_t)(&rbuf3);

	RxDescriptors[2].RDES0 = 0x80000000;
	RxDescriptors[2].RDES1 = 0x00080008;
	RxDescriptors[2].RDES2 = (uint32_t)(&rbuf4);
	RxDescriptors[2].RDES3 = (uint32_t)(&rbuf5);

	RxDescriptors[3].RDES0 = 0x80000000;
	RxDescriptors[3].RDES1 = 0x00088008;
	RxDescriptors[3].RDES2 = (uint32_t)(&rbuf6);
	RxDescriptors[3].RDES3 = (uint32_t)(&rbuf7);
}

void ETH_init() {
	ETH_MspInit(); // initialize low-level stuff (GPIO, clock, interrupts)

	while (*ETH_DMABMR & 0x00000001); // wait for software reset bit to be cleared before programming registers

	*ETH_DMABMR |= 0x04000000; // set mixed burst bit
	*ETH_DMAIER |= 0x0001A5FE; // set bits for enabled interrupts
	*ETH_DMAOMR |= 0x072000C0; // configure operation mode register

	// Create transmit and receive descriptor lists and write their addresses to DMARDLAR and DMATDLAR
	ETH_init_descriptors();
	*ETH_DMARDLAR = (uint32_t)(RxDescriptors);
	*ETH_DMATDLAR = (uint32_t)(TxDescriptors);

	*ETH_MACFFR |= 0x80000000; // receive all frames
}

void ETH_start() {
	*ETH_MACCR |= 0x0001488C; // enable transmit and receive on the MAC
	*ETH_DMAOMR |= 0x00002002; // enable transmit and receive on the DMA
}

// Configures the serial management interface (SMI), sets PHY address and MDC speed in MII address register
void ETH_config_SMI(uint8_t phy_addr) {
	*ETH_MACMIIAR |= (phy_addr & 0x1F) << 11; // PHY address should be 5 bits wide
	*ETH_MACMIIAR |= 0x00000010; // select highest HCLK range (HCLK = 216 MHz)
}

// Load PHY register address into MAC MII address register
void ETH_load_reg_addr(uint8_t reg_addr, uint8_t check_busy) {
	if (check_busy) {
		while (*ETH_MACMIIAR & 0x00000001); // wait until busy bit is cleared
	}

	if (reg_addr == 0x00) *ETH_MACMIIAR &= ~0x00000780; // cannot write zero using OR, must clear using AND
	else *ETH_MACMIIAR |= (reg_addr & 0x1F) << 6; // register address should be 5 bits wide
}

// Writes to a register on a PHY using the SMI (serial management interface)
void ETH_write_PHY(uint8_t reg_addr, uint16_t write_data, uint8_t check_busy) {
	ETH_load_reg_addr(reg_addr, check_busy);

	*ETH_MACMIIDR &= ~0x0000FFFF; // clear lower 16 bits of data register
	*ETH_MACMIIDR |= write_data; // write data to lower 16 bits of data register

	*ETH_MACMIIAR |= 0x00000002; // set write bit
	*ETH_MACMIIAR |= 0x00000001; // set busy bit to start write
}

uint16_t ETH_read_PHY(uint8_t reg_addr) {
	ETH_load_reg_addr(reg_addr, 1);

	*ETH_MACMIIAR &= ~0x00000002; // clear write bit
	*ETH_MACMIIAR |= 0x000000001; // set busy bit to start read

	while (*ETH_MACMIIAR & 0x00000001); // wait for read to complete
	return *ETH_MACMIIDR;
}

void ETH_InterruptRequestHandler() {
	if (*ETH_DMASR & 0x00010000) { // check if there was a normal interrupt
		if (*ETH_DMASR & 0x00000040) { // check if receive completed
			*ETH_DMASR &= ~0x00000040;
			*ETH_DMASR &= ~0x00010000;

			HAL_GPIO_TogglePin(Heartbeat_GPIO_Port, Heartbeat_Pin);

			// Give ownership of applicable receive descriptors back to DMA
			for (uint8_t i = 0; i < 2; i++) {
				if (~(RxDescriptors[i].RDES0) & 0x80000000) {
					RxDescriptors[i].RDES0 |= 0x80000000;
				}
			}
		}

		if (*ETH_DMASR & 0x00000004) { // check if transmit buffer unavailable
			*ETH_DMASR &= ~0x00000004;
			*ETH_DMASR &= ~0x00010000;

			// Give ownership of all transmit descriptors back to DMA
			for (uint8_t i = 0; i < 4; i++) {
				TxDescriptors[i].TDES0 |= 0x80000000;
			}

			*ETH_DMATPDR |= 0x1; // trigger transmit poll demand
		}
	}

	if (*ETH_DMASR & 0x00008000) { // check if there was an abnormal interrupt
		if (*ETH_DMASR & 0x00002000) { // check fatal bus error status bit
			*ETH_DMASR &= ~0x00002000;
			*ETH_DMASR &= ~0x00008000;

			if (*ETH_DMASR & 0x02000000) {
				*ETH_DMASR &= ~0x02000000;
				*ETH_DMASR &= ~0x00008000;
			}
			else {
				int x = 1;
			}

			if (*ETH_DMASR & 0x01000000) {
				*ETH_DMASR &= ~0x01000000;
				*ETH_DMASR &= ~0x00008000;
			}
			else {
				int x = 1;
			}

			if (*ETH_DMASR & 0x00800000) {
				*ETH_DMASR &= ~0x00800000;
				*ETH_DMASR &= ~0x00008000;
			}
			else {
				int x = 1;
			}
		}

		if (*ETH_DMASR & 0x00000100) { // check if receive process was stopped
			*ETH_DMASR &= ~0x00000100;
			*ETH_DMASR &= ~0x00008000;
		}

		if (*ETH_DMASR & 0x00000080) { // check if receive buffer unavailable
			*ETH_DMASR &= ~0x00000080;
			*ETH_DMASR &= ~0x00008000;
		}

		if (*ETH_DMASR & 0x00000020) { // check transmit underflow status bit
			*ETH_DMASR &= ~0x00000020;
			*ETH_DMASR &= ~0x00008000;
		}

		if (*ETH_DMASR & 0x00000010) { // check receive overflow status bit
			*ETH_DMASR &= ~0x00000010;
			*ETH_DMASR &= ~0x00008000;
		}

		if (*ETH_DMASR & 0x00000008) { // check transmit jabber timeout bit
			*ETH_DMASR &= ~0x00000008;
			*ETH_DMASR &= ~0x00008000;
		}

		if (*ETH_DMASR & 0x00000002) { // check if transmit process was stopped
			*ETH_DMASR &= ~0x00000002;
			*ETH_DMASR &= ~0x00008000;
		}
	}
}
