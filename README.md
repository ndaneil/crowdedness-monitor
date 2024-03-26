# Crowdedness Monitor

Every city has popular spots which can get crowded at unpredictable times. The crowdedness of a place, like how many people are at a study hall, in the office or at a park can be a major factor in deciding whether or not to go there. Information about crowdedness, however, is rarely available. This projects aims to solved this by using smart sensors listening to wireless signals and noise level to help determine how crowded a place is.

## Sensor overview

To detect the crowdedness of a place, I decided to use three sensors:
 - A microphone for noise level
 - Nearby WiFi devices
 - Nearby Bluetooth devices

My aim was to provide a prototype that works in several environments. For indoor places which people regularly visit, WiFi device data can be a good predictor as people connect their smartphones/laptops to the local network. For small places like cafés, noise level could correlate with crowdedness too. I chose to detect bluetooth devices as most people leave bluetooth on on their phones (e.g. to listen to music with wireless headsets) and scanning for bluetooth devices could be one way of measuring crowdedness in outdoor public places like parks.

## Bill Of Materials

The following components are required for the sensor:
 1. AVR-IoT Cellular Mini - this is the brain of the sensor, measuring noise levels and sending all data to the cloud using the built-in cellular modem
 2. ESP32, for this project, a LOLIN32 lite board - the microcontroller used to count the number of nearby WiFi and Bluetooth devices
 3. Microphone module - for measuring the ambient noise level
 4. Toggle switch
 5. TP4056 LiPo battery charger
 6. 2.54mm 40pin female header (x2)
 7. 2.54mm 4pin female header with longer leads
 8. Wires
 9. 2000mAh LiPo battery (other capacity works too)
 10. 2x male and 1x female micro jst connectors
 11. M2.5 screws (4x)
 12. 70x90mm prototyping board

<p align="center"><img src="./images/bom-all.png" width="80%"></p>

## Hardware assembly

Before starting to assemble the hardware, the first step should be activating the SIM card for the AVR-IoT Cellular Mini and testing that it successfully connects to Microchip Sandbox. For that, follow this Hackster guide: https://www.hackster.io/keenan-johnson/avr-iot-cellular-mini-107a63
If everything works, solder the headers onto the board and get start assembling the sensor.

The schematic for wiring the components is shown below:

<p align="center"><img src="./images/schematics.png" width="80%"></p>

The pinout for the ESP32 and AVR-IoT Cellular Mini boards are available here:
 - https://www.microchip.com/en-us/development-tool/ev70n78a
 - https://mischianti.org/esp32-wemos-lolin32-lite-high-resolution-pinout-and-specs/

Apart from the ground connection, the two microcontrollers are connected via a Serial RX-TX pair and an other GPIO connection. This can be used in case of an optional battery saving mode to wake up the ESP32. GPIO4 was chosen on the ESP32-side since it is an RTC pin, so it can be used to wake up the microcontroller. The other two pins for the serial connection were chosen from the ones that had no limitations. A good guide on what pins to use on an ESP32 can be found here: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

To make everything fit inside the custom casing I designed, I wired up the major components on a perfboard according to the image below:

<p align="center"><img src="./images/wiring.jpg" width="80%"></p>

Every wire is connected to the pin on the female header next to it. The green wires are the serial connection, blue is ground, red is 3.3V, white is the sound sensor's output and brown in the wakeup signal for the ESP32. I have also cut a hole on the right side to the perfboard in order to pass the battery wires through. After soldering the connections on the back of the perfboard, the components can be placed in their slots:

<p align="center"><img src="./images/components-placed.jpg" width="80%"></p>

The entire perfboard, along with the battery fits inside the casing I designed:

<p align="center"><img src="./images/case.png" width="80%"></p>

The STL files for the top and bottom parts are available [here](./3dfiles/case-top.stl) and [here](./3dfiles/case-bottom.stl). There are cutouts for the switch, the LiPo charging board and the microphone. I opted to include a separate charger besides the one onboard the AVR-IoT Cellular Mini board in order to be able to charge the battery while everything else is powered down. As can be seen in the schematic and on the image below too, the battery is connected to the TP4056's battery terminals through the female micro JST connector and the male micro JST connectors are connected to the protected output of the LiPo charging board through the switch. **Make sure to check if the polarity of the JST connectors is correct!** The ones I have have had reverse polarity which could have damaged both microcontrollers. I have used a tweezer to remove the two wires from the plastic connector and then swapped them.

<p align="center"><img src="./images/battery-wiring.jpg" width="80%"></p>

After soldering the wires together, the charger can be hot glued into place to the bottom of the case and the switch can be secured with a nut or a keycap from the outside.

<p align="center"><img src="./images/bat-placement.jpg" width="80%"></p>

Then the top part can be placed and secured witch screws. Removing the microcontrollers beforehand allows easier access to the holes for the M2.5 screws. The battery cables can also be connected to the microcontrollers.

<p align="center"><img src="./images/top-placement.jpg" width="80%"></p>

When assembled, the smart sensor looks like this:

<p align="center"><img src="./images/final-assembled.jpg" width="80%"></p>

Let's write the software next.

## Software

The software part can be divided into three parts:
 - Cloud data storage
 - Device firmware
 - Data visualization and presentation

All three steps are important and necessary to create a smart and useful sensor. For storing data in the cloud, I opted to use InfluxDB, a time-series database. It has a free, managed option as well as paid and self-hosted options making it suitable for several scenarios. In this guide, I will show how to use the InfluxDB's cloud solution. For writing the device firmwares, I used the Arduino IDE. Finally, for data visualization, I used Grafana.

### InfluxDB

### ESP32

### AVR IoT Cellular Mini

https://iot.microchip.com/docs/arduino/introduction/devenv

### Grafana

## Measurements



