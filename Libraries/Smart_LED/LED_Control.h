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

#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "cybsp.h"
#include <stdint.h>
#include <stdlib.h>

#include "../Smart_LED/Smart_LED.h"

#define LED_BRIGHTNESS_MULTIPLIER 		10
#define LED_BRIGHTNESS_DIVIDER 			12

#define RED_MIN_THRESHOLD				0
#define GREEN_MIN_THRESHOLD				3
#define BLUE_MIN_THRESHOLD				0

#define PULSING_TIME_TO_BRIGHTEST		128

extern STRIP_t 	strip;
extern uint16_t LED_Count;
extern uint16_t Chaser_Pos;

void limitLEDoutput(void);
void scaleLEDbrightness(void);
void setLEDcolor(uint16_t i, uint8_t red, uint8_t green, uint8_t blue);
void LEDOn(uint32_t i, int scaling);
void LEDOff(uint32_t i);


void InitChaser(void);
uint8_t FadingChaserLight(uint16_t uC_time);
uint8_t PulsingLight(uint16_t uC_time, uint16_t time_to_brightest);
void SteadyLight(void);
void OffLight(void);

void SetColorsFromCAN(uint8_t CAN_data[3]);
void SetTimingFromCAN(uint8_t CAN_timing);
void SetBitmaskFromCAN(uint32_t bitmask);

#endif /* LED_CONTROL_H */