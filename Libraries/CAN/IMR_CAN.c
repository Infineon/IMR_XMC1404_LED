/******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product").
 * By including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing so
 * agrees to indemnify Cypress against all liability.
******************************************************************************/

#include "IMR_CAN.h"
#include "LED_Control.h"
#include "IMR_CAN_GLOBAL.h"
#include <stdint.h>

uint16_t TimeStampLED = 0;
IMR_CAN_SENSOR_LED_COMMANDS_t current_mode = LED_MODE_OFF;
uint8_t CAN_DATA[8] = {0};

/***************************************************************************************************************/

void CAN_IRQ_RX_MESSAGE_HANDLER(void) {
	XMC_CAN_MO_Receive(&CAN_NODE_RECEIVE_LMO_NAME);			// Receive data from CAN Node and transfer into CAN data structure

	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RECEIVE_LMO_NAME);
	uint8_t *data = CAN_NODE_RECEIVE_LMO_NAME.can_data_byte;

		//Non optimal solution: Storing data in global object to make accessible in main
	for (int i = 0; i < 8; i++) {
		CAN_DATA[i] = data[i];
	}
	CAN_process_data(data);
}

void CAN_IRQ_RX_ALL_MESSAGE_HANDLER() {
	XMC_CAN_MO_Receive(&CAN_NODE_RECEIVE_ALL_LMO_NAME);

	uint32_t id = XMC_CAN_MO_GetIdentifier(&CAN_NODE_RECEIVE_ALL_LMO_NAME);
	uint8_t *data = CAN_NODE_RECEIVE_ALL_LMO_NAME.can_data_byte;

	for (int i = 0; i < 8; i++) {
			CAN_DATA[i] = data[i];
	}
	CAN_process_data(data);
}

void CAN_process_data(uint8_t *data) {
	SetColorsFromCAN(&data[1]);
	SetTimingFromCAN(data[4]);		// used only for some modes (e.g. pulse, chaser)

	uint32_t bitmask = (((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 8) | ((uint32_t)data[7]));
	uint32_t msb = (bitmask >> 23) << 31;
	//Calculate bitmask depending on board

	if ((board_id % 8 == 0 || board_id % 8 == 4) && XMC_GPIO_GetInput(CAN_ID6_PORT, CAN_ID6_PIN)) { //short board
			SetBitmaskFromCAN((bitmask >> 1) | ((uint32_t) 0x7fffffe0) | msb);
	}
	else if (board_id % 8 == 0 || board_id % 8 == 4 || board_id % 8 == 3 || board_id % 8 == 7){
			SetBitmaskFromCAN(bitmask | msb);
	}
	else if (board_id % 8 == 1 || board_id % 8 == 5) { //middle board front/back
		SetBitmaskFromCAN((bitmask >> 9) | ((uint32_t) 0x7fffffe0) | msb);
	}
	else { //left board front/back
		SetBitmaskFromCAN((bitmask >> 17) | ((uint32_t) 0x7fffffe0) | msb);
	}

	current_mode = data[0];
	TimeStampLED = 0;
	if (current_mode == LED_MODE_CHASER)
	{
		InitChaser();
	}

	XMC_CCU4_SLICE_StartTimer(TIMER_LED_HW);
}

/***************************************************************************************************************/

void CAN_Initialize() {
	uint32_t num = board_id % 8;
	uint32_t layer = board_id / 8;
	if (num <= 2) {
		XMC_CAN_MO_SetIdentifier(&CAN_NODE_RECEIVE_LMO_NAME, (uint32_t)(LED_LAYER_1_FRONT + (layer * 4) + 0));
	}
	else if (num == 3) {
		XMC_CAN_MO_SetIdentifier(&CAN_NODE_RECEIVE_LMO_NAME, (uint32_t)(LED_LAYER_1_FRONT + (layer * 4) + 1));
	}
	else if (num <= 6) {
		XMC_CAN_MO_SetIdentifier(&CAN_NODE_RECEIVE_LMO_NAME, (uint32_t)(LED_LAYER_1_FRONT + (layer * 4) + 2));
	}
	else {
		XMC_CAN_MO_SetIdentifier(&CAN_NODE_RECEIVE_LMO_NAME, (uint32_t)(LED_LAYER_1_FRONT + (layer * 4) + 3));
	}
	
	XMC_CAN_MO_SetIdentifier(&CAN_NODE_RECEIVE_ALL_LMO_NAME, (uint32_t)(LED_ALL));

	#if (CAN_TRANSCEIVER_STB_PIN_ENABLE)
		XMC_GPIO_SetOutputLow(CAN_STB_PIN_PORT_NAME, CAN_STB_PIN_PIN_NAME);	// Toggle CAN RX LED to indicate that a message has been received
	#endif

	NVIC_EnableIRQ(TIMER_LED_PERIOD_MATCH_EVENT_IRQN);			/* Enable CCU4 timer period match interrupt */
	NVIC_EnableIRQ(CHASER_TIMER_PERIOD_MATCH_EVENT_IRQN);
	NVIC_EnableIRQ(CAN_IRQ_RX_NUMBER);							// Enable NVIC IRQ with correct IRQ number - see Reference Manual
	NVIC_EnableIRQ(CAN_IRQ_RX_ALL_NUMBER);

	/* Change Interrupt event source of channel 16 (LED Timer) & channel 3 (CAN) ... see XMC1400 Reference Manual Table 5-1 */
	WR_REG(SCU_GENERAL->INTCR0, SCU_GENERAL_INTCR0_INTSEL3_Msk, SCU_GENERAL_INTCR0_INTSEL3_Pos,   0x02);    	// CAN RX message to board ID
	WR_REG(SCU_GENERAL->INTCR0, SCU_GENERAL_INTCR0_INTSEL4_Msk, SCU_GENERAL_INTCR0_INTSEL4_Pos,   0x02);		// CAN RX message to all boards
	WR_REG(SCU_GENERAL->INTCR1, SCU_GENERAL_INTCR1_INTSEL16_Msk, SCU_GENERAL_INTCR1_INTSEL16_Pos, 0x02);    	// CAN TimeOut Timer
}
