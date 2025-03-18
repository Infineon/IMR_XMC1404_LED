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

#include "LED_Control.h"
#include "cycfg_peripherals.h"
#include <stdint.h>

STRIP_t 	strip;
uint16_t 	LED_Count = 0;
uint16_t 	Chaser_Pos = 0;

uint8_t CAN_RED;
uint8_t CAN_GREEN;
uint8_t CAN_BLUE;

uint16_t pulse_timing = PULSING_TIME_TO_BRIGHTEST;	// timing for pulse
uint32_t led_bitmask = 0;	// Bitmask for individual LED control (works for PULSE and STEADY): 1: LED off, 0: LED on
uint32_t ignore = 0;

uint8_t temp_pulsing_red = 0;
uint8_t temp_pulsing_green = 0;
uint8_t temp_pulsing_blue = 0;


/*******************************************************************************
 * Function:      setLEDcolor
 * Author:        Matthias Brandl
 *
 * Creation:      5.5.2017
 * Last Modified: 5.5.2017, MaBr
 * Description:   sets the color LEDs to one of 12 colors
 ******************************************************************************/
void setLEDcolor(uint16_t i, uint8_t red, uint8_t green, uint8_t blue) {
	strip.leds[i].red = red;
	strip.leds[i].green = green;
	strip.leds[i].blue = blue;
}

/*******************************************************************************
 * Function:      limitLEDoutput
 * Author:        Michael Schmidt
 *
 * Creation:      08.08.2023
 * Last Modified: 08.08.2023, SchmidtMicha
 * Description:   limits the minimum Output level of the LEDs.
 ******************************************************************************/
void limitLEDoutput(void) {
	for (uint16_t i = 0; i < LED_Count; i++) {			// Limit LED Output according to Thresholds
		if (strip.leds[i].blue <= BLUE_MIN_THRESHOLD) strip.leds[i].blue = 0;
		if (strip.leds[i].red <= RED_MIN_THRESHOLD) strip.leds[i].red = 0;
		if (strip.leds[i].green <= GREEN_MIN_THRESHOLD) strip.leds[i].green = 0;
	}
}

/*******************************************************************************
 * Function:      scaleLEDbrightness
 * Author:        Matthias Brandl
 *
 * Creation:      9.5.2017
 * Last Modified: 9.5.2017, MaBr
 * Description:   scales the brightness of the LEDs.
 ******************************************************************************/
void scaleLEDbrightness(void) {
	for (uint16_t i = 0; i < LED_Count; i++) {			// Scale LED Brightness
		strip.leds[i].blue 	= strip.leds[i].blue  * LED_BRIGHTNESS_MULTIPLIER / LED_BRIGHTNESS_DIVIDER;  // Detzel: the constants are ints, so this line has no effect (muliply with 1)?
		strip.leds[i].red 	= strip.leds[i].red   * LED_BRIGHTNESS_MULTIPLIER / LED_BRIGHTNESS_DIVIDER;
		strip.leds[i].green = strip.leds[i].green * LED_BRIGHTNESS_MULTIPLIER / LED_BRIGHTNESS_DIVIDER;
	}
}

/*******************************************************************************
 * Function:      initSnake
 * Author:        Matthias Brandl, Michael Schmidt
 *
 * Creation:      9.5.2017
 * Last Modified: 08.08.2023, SchmidtMicha
 * Description:   initalizes the start positions of the snake.
 ******************************************************************************/
void InitChaser() {
	// Reset Chaser Position to Zero
	Chaser_Pos = 0;

}

/*******************************************************************************
 * Function:      FadingChaserLight
 * Author:        Matthias Brandl, Michael Schmidt
 *
 * Creation:      9.5.2017
 * Last Modified: 08.08.2023, SchmidtMicha
 * Description:   controls the LEDs to output a moving and fading chaser light
 ******************************************************************************/
uint8_t FadingChaserLight(uint16_t uC_time) {
	if (Chaser_Pos == uC_time)
	{
		setLEDcolor(Chaser_Pos++, CAN_RED, CAN_GREEN, CAN_BLUE);
		if (Chaser_Pos >= 2)
		{
			setLEDcolor(Chaser_Pos-2, 0, 0, 0); // Turn of previous LED
		}
	}
	//scaleLEDbrightness();
	return (uC_time >= LED_Count)?1:0;
}

void LEDOn(uint32_t i, int scaling) {
	setLEDcolor(i, (uint8_t)(CAN_RED / scaling), (uint8_t)(CAN_GREEN / scaling), (uint8_t)(CAN_BLUE / scaling));
}

void LEDOff(uint32_t i) {
	setLEDcolor(i, 0, 0, 0);
}

/*******************************************************************************
 * Function:      PulsingLight
 * Author:        Matthias Brandl, Michael Schmidt
 *
 * Creation:      9.5.2017
 * Last Modified: 08.08.2023, SchmidtMicha
 * Description:   controls the LEDs to output a pulsing light
 ******************************************************************************/
uint8_t PulsingLight(uint16_t uC_time, uint16_t time_to_brightest) {

	for (uint16_t i = 0; i < LED_Count; i++) {			// Scale LED Brightness
		if ((~led_bitmask & 1<<i))
		{
			if (uC_time <= time_to_brightest)
				setLEDcolor(i, CAN_RED * ((float) uC_time / time_to_brightest), CAN_GREEN * ((float) uC_time / time_to_brightest), CAN_BLUE * ((float) uC_time / time_to_brightest));
			else
				setLEDcolor(i, CAN_RED * ((float) ((time_to_brightest * 2) - uC_time) / time_to_brightest), CAN_GREEN * ((float) ((time_to_brightest * 2) - uC_time) / time_to_brightest), CAN_BLUE * ((float) ((time_to_brightest * 2) - uC_time) / time_to_brightest));
		}else{
			setLEDcolor(i, 0, 0, 0);
		}
	}
	return (uC_time >= (time_to_brightest * 2))?1:0;
}

/*******************************************************************************
 * Function:      SteadyLight
 * Description:   controls the LEDs to output a steady light
 ******************************************************************************/
void SteadyLight(void) {
	for (uint16_t i = 0; i < LED_Count; i++) {
		if ((~led_bitmask & 1<<i))
		{
			setLEDcolor(i, CAN_RED, CAN_GREEN, CAN_BLUE);
		}else{
			if (!ignore) {
				setLEDcolor(i, 0, 0, 0);
			}
		}
	}
}

/*******************************************************************************
 * Function:      OffLight
 * Author:        [Your Name]
 *
 * Creation:      [Creation Date]
 * Last Modified: [Last Modified Date]
 * Description:   Turns off all LEDs by setting their color values to zero.
 ******************************************************************************/
void OffLight(void)
{
    for (uint16_t i = 0; i < LED_Count; i++) {			// Set all LEDs to Zero
        strip.leds[i].blue = 0;
        strip.leds[i].red = 0;
        strip.leds[i].green = 0;
    }
}

/*******************************************************************************
 * Function:      SetColorsFromCAN
 * Author:        Jonas Granig
 * Creation:      02.04.2024
 * Last Modified: 02.04.2024, JonasGranig
 * Description:   setting LED color from 3 Byte CAN Data.
 ******************************************************************************/
void SetColorsFromCAN(uint8_t CAN_data[3]) {
    CAN_RED = CAN_data[0];
    CAN_GREEN = CAN_data[1];
    CAN_BLUE = CAN_data[2];
}

/*******************************************************************************
 * Function:      GetColors
 * Author:        Samuel Detzel
 * Creation:      2025-02-19
 * Last Modified: 2025-02-19, Samuel Detzel
 * Description:   Retrieves the current LED color values.
 ******************************************************************************/
void GetColors(uint8_t colors[3]) {
    colors[0] = CAN_RED;
    colors[1] = CAN_GREEN;
    colors[2] = CAN_BLUE;
}

/*******************************************************************************
 * Function:      SetTimingFromCAN
 * Author:        Samuel Detzel
 * Creation:      2025-02-19
 * Last Modified: 2025-02-19, Samuel Detzel
 * Description:   Sets the timing for the LEDs based on CAN data.
 ******************************************************************************/
void SetTimingFromCAN(uint8_t CAN_timing){
    XMC_CCU4_SLICE_SetTimerPeriodMatch(TIMER_LED_HW, 150*CAN_timing);
    XMC_CCU4_EnableShadowTransfer(ccu4_0_HW, XMC_CCU4_SHADOW_TRANSFER_SLICE_0);
}

/*******************************************************************************
 * Function:      SetBitmaskFromCAN
 * Author:        Samuel Detzel
 * Creation:      2025-02-19
 * Last Modified: 2025-02-19, Samuel Detzel
 * Description:   Sets the bitmask for individual LED control based on CAN data.
 ******************************************************************************/
void SetBitmaskFromCAN(uint32_t bitmask)
{
    led_bitmask = bitmask;
    ignore = bitmask >> 31;
}
