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

#ifndef LIBRARIES_IMR_CAN_H_
#define LIBRARIES_IMR_CAN_H_

#include "IMR_CAN_GLOBAL.h"
#include "cybsp.h"
#include "cy_utils.h"

#define CAT(x, y) CAT_(x, y)
#define CAT_(x, y) x ## y
#define CAN_TX_TIMEOUT		0x400	// Timeout counter value ... 1024

extern uint16_t TimeStampLED;
extern IMR_CAN_SENSOR_LED_COMMANDS_t current_mode;
extern uint8_t CAN_DATA[8];
extern uint32_t board_id;

void CAN_Initialize();

void CAN_IRQ_RX_MESSAGE_HANDLER(void);
void CAN_IRQ_RX_ALL_MESSAGE_HANDLER(void);
void CAN_process_data(uint8_t *data);

uint32_t XMC_CAN_MO_Busy(XMC_CAN_MO_t* mo_ptr);

// Select the CAN_Node name chosen in the MTB Device Configurator;
#define CAN_NODE_CONFIGURATOR_NAME				CAN_NODE
// Select the CAN_Node number chosen in the MTB Device Configurator;
// CAN Node 0 ... CAN_NODE0		CAN Node 1 ... CAN_NODE1
#define CAN_NODE_CONFIGURATOR_CHANNEL			CAN_NODE1

/* IRQ Event Source Names for XMC1404 -
 * see XMC1400 Reference Manual Table 5-1 */
/* CAN Interrupt Number Setting - XMC1404 = IRQ3_IRQn */
#define CAN_IRQ_RX_NUMBER               		IRQ3_IRQn
/* CAN Interrupt Handler Setting - XMC1404 = IRQ3_Handler */
#define CAN_IRQ_RX_MESSAGE_HANDLER         		IRQ3_Handler
#define CAN_IRQ_RX_ALL_NUMBER					IRQ4_IRQn
#define CAN_IRQ_RX_ALL_MESSAGE_HANDLER			IRQ4_Handler

#define CAN_NODE_GLOBAL_HW_NAME			CAT(CAN_NODE_CONFIGURATOR_NAME, _HW)
#define CAN_NODE_RECEIVE_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_0)
#define CAN_NODE_TRANSMIT_LMO_NAME		CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_1)
#define CAN_NODE_RECEIVE_ALL_LMO_NAME	CAT(CAN_NODE_CONFIGURATOR_NAME, _LMO_2)

/*****************************************************************************/
/********************* SENSOR LED BOARD - IMPORTANT NOTICE *******************/
/*****************************************************************************/

/* This library is designed to work with the IMR2 CAN identification in mind;
 * CAN node IDs are set using DIP switches according to
 * the selected board type;
 * To ensure the correct pins are used, the naming for each
 * individual CAN ID pin must follow the same naming convention:
 * CAN_ID + PinNumber: e.g. CAN_ID1, CAN_ID2, ...
 *
 * Please make sure this naming convention is followed
 * when naming the corresponding pins in the MTB Device Configurator;
 *
 * Make sure to select up to 2 message objects in the global CAN setting
 * for the selected node in the MTB Device Configurator;
 * 		Set the Logical MO - LMO_0 to be the RECEIVE channel (RX)
 * 			Activate "RX Message Object SR"
 * 			Activate "RX Event" with "Rx Event SR Line" set to NVIC SR0
 *
 * 		Set the Logical MO - LMO_1 to be the TRANSMIT channel (TX)
 * 		Make sure to deactivate the option to "Store Config in Flash"
 *
 *****************************************************************************/

/*****************************************************************************/
/***************** DO NOT CHANGE SETTINGS ABOVE THIS LINE ********************/
/*****************************************************************************/
// Select if the board is supposed to receive CAN messages;
// 0U ... CAN Receive NOT used;		1U ... CAN Receive used;
#define CAN_NODE_RECEIVE_ENABLE			(1U)
// Select if the board is supposed to transmit CAN messages;
// 0U ... CAN Transmit NOT used;	1U ... CAN Transmit used;
#define CAN_NODE_TRANSMIT_ENABLE		(1U)
// Select if a specific LED should indicate the receive of a CAN message;
// 0U ... CAN RX LED NOT used; 		1U ... CAN RX LED used;
#define CAN_NODE_RECEIVE_LED_ENABLE		(0U)
// Select if the Transceiver has a STB shutdown pin that is being used;
// 0U ... STB Pin NOT used;			1U ... STB Pin used;
#define CAN_TRANSCEIVER_STB_PIN_ENABLE	(1U)

#if (CAN_NODE_RECEIVE_LED_ENABLE)
// Select the CAN RX LED Pin name chosen in the MTB Device Configurator;
#define CAN_RX_LED_PIN_CONFIGURATOR_NAME	LED_RED
#endif

#if (CAN_TRANSCEIVER_STB_PIN_ENABLE)
// Select the STB Pin name chosen in the MTB Device Configurator;
#define CAN_STB_PIN_CONFIGURATOR_NAME		CAN_NODE_STB
#endif

/*****************************************************************************/
/***************** DO NOT CHANGE SETTINGS BELOW THIS LINE ********************/
/*****************************************************************************/

#if (CAN_NODE_RECEIVE_LED_ENABLE)
#define CAN_RX_LED_PIN_PORT_NAME	CAT(CAN_RX_LED_PIN_CONFIGURATOR_NAME, _PORT)
#define CAN_RX_LED_PIN_PIN_NAME		CAT(CAN_RX_LED_PIN_CONFIGURATOR_NAME, _PIN)
#endif

#if (CAN_TRANSCEIVER_STB_PIN_ENABLE)
#define CAN_STB_PIN_PORT_NAME		CAT(CAN_STB_PIN_CONFIGURATOR_NAME, _PORT)
#define CAN_STB_PIN_PIN_NAME		CAT(CAN_STB_PIN_CONFIGURATOR_NAME, _PIN)
#endif

/* Interrupt event source names - see XMC1400 Reference Manual Table 5-1 */
/* Defines IRQ number of the period match event interrupt */
#define TIMER_LED_PERIOD_MATCH_EVENT_IRQN 				IRQ16_IRQn
/* Defines handler of the period match event interrupt */
#define TIMER_LED_PERIOD_MATCH_EVENT_HANDLER 			IRQ16_Handler

#define CHASER_TIMER_PERIOD_MATCH_EVENT_IRQN			IRQ22_IRQn
#define CHASER_TIMER_PERIOD_MATCH_EVENT_HANDLER			IRQ22_Handler

#endif /* LIBRARIES_IMR2_CAN_H_ */
