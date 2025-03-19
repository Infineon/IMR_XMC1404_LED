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

#include "../Smart_LED/Smart_LED.h"

const uint16_t bit_1 = 0x33;
const uint16_t bit_0 = 0x1a;

void InitRGB(STRIP_t* strip, XMC_GPIO_PORT_t* port, uint8_t pin, uint8_t size)
{
	strip->port = port;
	strip->pin = pin;
	strip->size = size;
	strip->leds = malloc(sizeof(RGB_t) * size);

	XMC_GPIO_CONFIG_t config =
	{
		.mode = XMC_GPIO_MODE_OUTPUT_PUSH_PULL,
		.output_level = XMC_GPIO_OUTPUT_LEVEL_LOW,
	};
	XMC_GPIO_Init(strip->port, strip->pin, &config);

	for (uint8_t i=0; i<size; i++)
	{
		strip->leds[i].red = 0;
		strip->leds[i].green = 0;
		strip->leds[i].blue = 0;
	}
}

void SendRGB(STRIP_t* strip)
{
	//clear output
	strip->port->OMR = 0x10000U << strip->pin;

	for (int i = 0; i < strip->size; i++)
	{
		for (int8_t j=23; j>=0; j--)
		{
			if (*((uint32_t*)&strip->leds[i]) & (1<<j))
			{
				// send one
				strip->port->OMR = (uint32_t)0x1U << strip->pin;
				__ASM (
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
					);
				strip->port->OMR = 0x10000U << strip->pin; \
			}
			else
			{
				// send zero
				strip->port->OMR = (uint32_t)0x1U << strip->pin; \
				__ASM (" NOP\n\t");
				strip->port->OMR = 0x10000U << strip->pin;  \
				__ASM (
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
						" NOP\n\t"
				);
			}
		}
	}
}
