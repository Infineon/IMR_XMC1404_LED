<!--
SPDX-FileCopyrightText: Copyright (c) 2024 Infineon Technologies AG
SPDX-License-Identifier: MIT
-->

# IMR Software for XMC1404 LED Control 

<a href="https://www.infineon.com">
<img src="./assets/images/Logo.svg" align="right" alt="Infineon logo">
</a>
<br>
<br>

## Overview

<p>This is the official Infineon GitHub repository for ModusToolbox™ software used with the demo board for IMR LED control.</p>
<p>The IMR LED control is responsible for receiving commands via CAN and lighting up the series of LEDs according to the mode and color request. Its main purpose is to provide attractive appearance for the IMR and to serve as user interface e.g. as motor speed indicator.</p>

### Features

- 5 V 32-bit microcontroller Arm® Cortex®-M0 XMC1404 48 MHz core frequency and 96 MHz peripheral clock with 200 kB Flash and 16 kB RAM
- CAN bus communication with onboard CAN transceiver
- Onboard DIP switch for unique board identification
- Up to 23 RGB LEDs in series for various innovative LED patterns

### Reference hardware

<p>This software is meant to run on following reference hardware:</p>
- <a href="https://www.infineon.com/cms/en/applications/robotics/development-platform/#!?fileId=8ac78c8c8eeb092c018f867fd8bc5414">DEMO_IMR_LED_USHAPE_V1 - Demo board for LED control with U-shape (long board)</a><br>
- <a href="https://www.infineon.com/cms/en/applications/robotics/development-platform/#!?fileId=8ac78c8c8eeb092c018f867fd1835410">DEMO_IMR_LED_ISHAPE_V1 - Demo board for LED control with I-shape (short board)</a>

#### Featured Infineon Products 
<p>Following products are featured by the reference hardware:
<br>
<br>
<table style="width:100%">
  <tr>
    <th>Product</th>
    <th>Description</th>
  </tr>
  <tr>
    <td><a href="https://www.infineon.com/cms/en/product/microcontroller/32-bit-industrial-microcontroller-based-on-arm-cortex-m/32-bit-xmc1000-industrial-microcontroller-arm-cortex-m0/xmc1404-q048x0200-aa/">XMC1404-Q048X0200 AA</a></td>
    <td>Low-cost good performance microcontroller with MATH co-processor for CORDID and HW Divide</td>
  </tr>
  <tr>
    <td><a href="https://www.infineon.com/cms/en/product/transceivers/automotive-transceiver/automotive-can-transceivers/tle9351bvsj/">TLE9351BVSJ</a></td>
    <td>High speed CAN transceiver for CAN and CAN FD</td>
  </tr>
</table>
</p>
<br>

## Getting started

### How to import and use this repository
<ol>
<li> Install and start ModusToolbox™ and select a workspace to be used (tested with Version 3.3, and 3.4).
<li> Import the project with the import wizard by pressing 'File' – 'Import…'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_1.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Select 'ModusToolbox™' – 'Import Existing Application In-Place' and press 'Next'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_2.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Find the Project Location by pressing 'Browse…'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_3.png" >
    </picture>
    <br>
    &nbsp;
</li>
<li> Select the project folder accordingly and press 'Finish'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_4.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Wait until the project is fully imported. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_5.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Notice that additional folder 'mtb_shared' should be created (if there was none) when the import is completed. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_6.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Right click the project folder and select 'ModusToolbox™' followed by 'Library Manager 2...'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_7.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Press the 'Update' button <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_8.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> When the Update is completed the sucessful messages should be displayed. If the update failed, try it again by repressing the 'Update' button. If this also fails try to clean the project, before trying it again. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_9.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Before building the project it is recommended to clean it by pressing 'Clean Application'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_10.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Finally the project can be compiled by pressing 'Build Application'. <br><br>
    <picture>
        <img src="./assets/images/MTB_Import_11.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Before flashing the project onto the board, connect the XMC™ Link Programming adapter using the 10-pin flat cable and <a href="./assets/DEMO_IMR_PROGADPTR_V1@e7eacb3013a-zip">the programming adapter</a> to provide power to the board. If the programming adapter is not available, connect external power supply to the 5V and GND pins of the green / P1 connector onboard. <br><br>
	<picture>
        <img src="./assets/images/MTB_Import_12.png">
    </picture>
    <br>
	<picture>
        <img src="./assets/images/MTB_Import_13.png">
    </picture>
    <br>
    &nbsp;
</li>
<li> Finally to flash the project onto the board, use the green play button in ModusToolbox™ on the bottom left - 'Quick Panel' - 'Launches' - 'IMR_XMC1404_LED Program (JLink)' to initiate the process.<br><br>
	<picture>
        <img src="./assets/images/MTB_Import_14.png">
    </picture>
    <br>
    &nbsp;
</li>
</ol>

## Additional information

Precise definition of the software and its features can be found in the close-to-code documentation on top of each file, at the specific function itself and in the software documentation.

### Related resources

- [Robotics development platform: Infineon Mobile Robot (IMR)](https://www.infineon.com/cms/de/applications/robotics/development-platform/)
- [IMR main control](https://www.infineon.com/cms/en/product/evaluation-boards/demo_imr_mainctrl_v1/)
- [IMR motor control](https://www.infineon.com/cms/en/product/evaluation-boards/demo_imr_mtrctrl_v1/)

### Licensing

Please see our [LICENSE](LICENSE) for copyright and license information.
