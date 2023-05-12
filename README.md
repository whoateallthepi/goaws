# goaws : a TinyGo Open Automatic Weather Station on Raspberry Pi Pico
## Installation
The bit where you are hoping for some helpful installation instructions. At the moment you are on your own. Compile it and get the debugger ready.
## Information
This is not an out-the-box solution. You are going to need to do a bit of work to get up and running. A soldering iron is essential and there will be problems to solve. 

The below is as much for my benefit as anyone - so I can remember how this works next time it breaks.

*Tom Cooper*
May 2023

## Overview
The code is a functioning weather station Pico 'head end' deisgned to report wind speeds and direction, rainfall, barometric pressure, humidity and temperature at a chosen report interval.
The station also keep a handful of daily totals (such as rainfall).
Communication is two way with the weather station sending out reports and processing, for example, time zone mesages from the base station (more on the basestation below).
The system is designed to communicate with a loraWan network such as the Things Network. 
The latest iteration of the code relies on a DS3231 clock. Development was done with a Wavesure, pico-RTC-DS3231
### Basestation
A sample python implementation of a basestation is available at https://github.com/whoateallthepi/weatherbasestation

## Wiring and hardware
### Pico
The pico is stacked with a DS3231 realtime clock set to UTC. See https://www.waveshare.com/wiki/Pico-RTC-DS3231
The alarm is not used.
#### Connections

Note the GND connections could be to any of the pico ground pins

| Pin number | Pico name     | Connection | Notes | constant value (if used) |
| ----- | ----     | ---------- | ----- | ------------------ |
| 1     | UART0 TX | to RAK811 RX | Direct connection |  |
| 2     | UART0 RX | to RAK811 TX | Direct connection |  |
| 3     | GND      | general earth      | Used as a ground for the wind sensor etc | |
| 4     | GP2      | to Wind      | Pin pulled up and interrupt fired when connected to GND via 1kohm |  windSpeedPin |
| 5     | GP3      | to Rain | Pin pulled up and interrupt fired when connected to GND via 1kohm |   rainPin |
| ... |
| 17    | GP13     | to red LED | TX   | led0  |
| 18    | GND      | used for LEDs |   | 
| 19    | GP14     | to green LED |     status         |  led1 |
| 20    | GP15     | to yellow LED |  RX  | led2 |
| 21    | SPIO RX  | to BME280 | Pin names vary on sensor boards - look for SDO | |
| 22    | SPIO CSn | to BME280 | Typically CSB/CS - chip select | |
| 23    | GND      | to BME280 | To ground on board | |
| 24    | SPIO SCK | to BME280 | SCL/SCK | |
| 25    | SPIO TX  | to BME280 | SDA/SDI | |
| 29    | GP22      | to base of NPN transistor | Setting high will reboot (emitter is earthed)| |
| 30    | RUN       | to collector of NPN transistor | |
| ... |
| 31    | ADCO     | Wind direction | Connected via voltage splitter to weather meters - see notes below | WIND_DIR|
| 32    | ADC1      | Battery level | Via voltage divider to input voltage - use ?? | |
| ... |
| 36    | 3V3 OUT  | various  |Source of power for RAK811, BME280 and ADC voltage | |
| 38    | GND      |  various | General ground for RAK811, IRQs etc| | 

#### Voltage dividers

To be done

#### Sparkfun weather meters
See https://cdn.sparkfun.com/assets/d/1/e/0/6/DS-15901-Weather_Meter.pdf
 
### Rak811 - lora
This version uses a RAK811 LoRa breakout modem. Hopefully this will be replaced soon with an
SPI connected device. The code is written to be fairly device independent so a alternative device/driver implementation should be re;atively easy.
See https://docs.rakwireless.com/Product-Categories/WisDuo/RAK811-Breakout-Board/Datasheet/
#### Wiring
The RAK811 just needs its RX and TX pins wiring in to the picos TX and RX pins, plus a link to the 3v line and earth.
#### Lora setup
Configure the RAK811 with the appropriate setting for your loraWan network before you start - this isn't done from this code. 

### Some useful AT commands for RAK811


at+get_config=lora:status

Airtime calculator https://avbentem.github.io/airtime-calculator/ttn/eu868/49

#### Clock setup
Set the DS3231 to UTC before you start - this isn't done in the code.


## Pico processing and design
goaws is coded in TinyGo. The original C version was multicore, but at the moment this isn't available on 
TinyGo. The pico's inbuild RTC is also obscured so a simple channel ticker is used to keep track of time.
GPIO interrupts monitor wind speed and rain gauge tips. The gauge tips are communicated by a channel,
the windspeed via a runtime/volatile call. This is to avoid clogging the channels. 
 
### collecting data
**Rain** is recoded via a gpio interrupt callback. Each click of the tipping gauge is rainMultiplier mms of rain. 

**Wind speed** operates via a similar interrupt with one click per second representing windMultiplier km/h. 
**Wind direction** is via an ADC. This is kept in radians to assist in calculating average directions.
The only other trick is to make sure the various array indexes are updated correctly so clicks are recorded in the right array position. 

### transmitting/receiving data
To come


**Incoming Mesages** are receivable for a brief period after transmission. The RAK811 is configured as a Class A loraWan device. 


**Outgoing messages** is largely about coercing variable into hex-byte strings and stuffing them at the 
RAK811.

### Drivers ###
A handful of drivers were written to support the development. See https://github.com/whoateallthepi/tinygodrivers. These are:

| Driver | Notes |  
| ------ | ----- | 
| [bme280spi](https://github.com/whoateallthepi/tinygodrivers/tree/main/bme280spi)     | This is an SPI version of the TinyGo I2C driver at https://github.com/tinygo-org/drivers 
| [ds3231](https://github.com/whoateallthepi/tinygodrivers/tree/main/ds3231) | Originally developed to add alarm funtion to existing driver. The alarm function worked but in the end I used the internal pico clock with frequent resync to the ds3231 |
| [ledpanel](https://github.com/whoateallthepi/tinygodrivers/tree/main/ledpanel) | Not really a driver but is used to flash the front panel leds |
| [rak8nn](https://github.com/whoateallthepi/tinygodrivers/tree/main/rak8nn) | This simplifies interaction with the modem. The driver implements an interface that supports basic comms methods ( Initialise(), Join(), Send() and Status()) so we can switch to another modem (the rak811 is already obsolete, but I have a few!)

### History
My original weather station was developed on an Arduino using the Sparkfun weather shield, weather meters and the code supplied by Sparkfun. I added a serial data link to this (instead of WiFi) and this has served well for 8+ years. Over that time I have fiddled a lot with the code and changed much hardware as various serial links failed or became obsolete. 

The launch of the pico gave me a chance to start from scratch (with several years' more experience). This was also an opportunity to assess if LoRa would pprovide a suitable (future proof?) data links. I am also more confident working in C on microcontrollers rather than Arduino's C++ dialect.

The use of the pico allows the use of an onboard real time clock, plus many of the timer and alarm features on the pico which are not available on the Arduino.

With the release of TinyGo, I took the opportunity to do a full redevelopment, from scratch. The original C code had 'evolved' and was hard to maintain and add / change sensors. 

The C code is still available at https://github.com/whoateallthepi/picoweatherstation

## Message types and handling
Note this is a Class A loraWAN device. Incoming messages (downlinks, in loraWAN speak), are received in a brief period following an uplink. Uplinks only happen every reportFrequency minutes (I use 10mins), so you will have to wait for responses.
### Currently implemented message types

| Message number | Hex | Details | Direction |
| -------------- | --- | ------- | --------- |
| 100            | x64 | Contains the latest weather readings | Uplink weather station > loraWAN |
| 101            | x65 | Details of station including date, time, altitude and position | Uplink |
| .. |
| 200            | xc8 | Set time zone - sends the current base station timezone to weather station, plus a seconds adjustment for the station clock (-127s to +127s)| Downlink loraWAN > weather station |
| 201            | xc9 | Set station details - sends the station details - altitude, position - to the weather tation |  Downlink |
| 202            | xca | Request station details - sends a request for the weather station to send a message 101 |  Downlink |
| 203            | xcb | Request softwarre reset - asks the station to do a reset by earthing RUN ) |  Downlink |

## Interaction with loraWAN
This is fairly basic. When the station boots, it tries to join the network with an 'at+join' command. If this fails (either bad parameters or no gateway available) it is retried a   number of times (see the rak8nn driver). If no connection is made, there is no recovery (at the moment) so I reboot.

Note the RAK811 should have been set up with the appropriate loraWAN parameters - device IDS etc - before connecting up. I do this with a serial terminal. See below.

**Outgoing mesages** (uplinks) are currently just the regular weather reports (100) and a confirmation message of the station details (101).  After 'OK' is received, the rak8nn driver keeps checking the UART for  a few milliseconds. This is to pick up any **incoming messages** (downlinks) from the network. These are processed as per the table above. At the moment, try not to send more than one downlink per reporting cycle - for example if you have sent a set sttation details message (201), wait until the next reporting cycle to send a timezone (200) message.

### Set up sequence for loraWAN on RAK811 ###

at+set_config=lora:work_mode:0 # Switch to lowrawan mode

at+set_config=lora:join_mode:0 # OTAA activation

at+set_config=lora:class:0 # class A

at+set_config=lora:region:EU868

at+set_config=lora:dev_eui:nnnnnnnnnnnnnnnn

at+set_config=lora:app_eui:nnnnnnnnnnnnnnnn

at+set_config=lora:app_key:<32 characters> 

at+set_config=device:restart

at+get_config=lora:status # to check...

at+join # Connect to lorawan - may take a few tries (up to 5)

at+send=lora:2:1234567890ABFF # try sending some data

## LED sequences

The pico reports various statuses and errors by flashing LEDs on the station panel. Three leds are available Red (led0), Green (led1) and Yellow (led3). Their basic roles are TX (transmit, red) Status (green), RX (receive, yellow). All three are flashed at boot - just as a wiring check. Below is the full list roughly in the order you would expect on a boot. The error states are in **bold**:

| state | led0 TX (red) | led1 STATUS (green) | led2 RX (yellow) | flashes | Notes|
| ----- | --- | --- | --- | ------- | ---------------- |
| *Boot sequence*|
|boot   | * | * | * | 1 | on boot |
|sensor check |   | * |   | 2 | Pressure/humidity/temperature sensor connected |
| **sensor fail** | * | * | | 4 | Error initialising/connection to pressure/humidity/temperature sensor |
| network OK | * |   | * | 1 | joined loraWan |
| **network fail** | * | | * | 4 | failed to join network |
| *While running* |
| RX success | | |  * | 1 | Correctly formatted message received |
| **RX fail** | | |  * | 4 | Error processing incoming message - usually a corrupt/unexpected message |
| TX success | * | | | 1 | Successful uplink to gateway |
|**TX fail** | * | | | 4 | Transmit fail - either no response from modem or an error code |
| Alive | | * | | 1 | Flashed evey 15s just for reassurance |
| **Alive - no network** | * | * |   | 1 | Every 15s - no network but working

