# RC transmitter for cars, boats, tanks and simple model airplanes
Simple surface 5 channel RC transmitter.
The hardware includes nRF24L01+ transceiver and ATmega328P processor.
Telemetry monitors receiver voltage using LED indication.
The code is Arduino.

This RC transmitter works with RC receiver from my repository [**RX_nRF24_Motor_Servo**](https://github.com/stanekTM/RX_nRF24_Motor_Servo)

Thank you to "Phil_G" http://www.singlechannel.co.uk for the calibration and reverse routine I used in the code.

## The firmware includes
### LED mode:
* Normal mode = LED TX is lit
* TX battery voltage 1S LiPo (4.2V) < TX monitored voltage = TX LED flash at a interval of 0.2s
* RX battery voltage 1S LiPo (4.2V) < RX monitored voltage = TX, RX LEDs flash at a interval of 0.5s
* RX receiver off or signal loss = TX LED flash at a interval of 0.1s
### Calibration:
* Hold calibration button, switch transmitter TX on, still holding calibration button move all controls to extremes including auxilliary pots.
* Center all controls and aux pots.
* Release calibration button (saved to eeprom).
### Servo reversing:
* To reverse the desired channel, hold the joystick in the end position and turn on the TX transmitter (saved to eeprom).

## Arduino pins
```
A0 - joystick 1
A1 - joystick 2
A2 - joystick 3
A3 - joystick 4
A4 - potentiometer 5

D6 - LED
D4 - calibration button (I had to add a 10k resistor -> VCC even when the internal INPUT_PULLUP is activated)
A7 - input TX battery

nRF24L01:
D9  - CE
D10 - CSN
D11 - MOSI
D12 - MISO
D13 - SCK
```

## Used libraries
* <RF24.h>   https://github.com/nRF24/RF24
* <EEPROM.h> Arduino standard library
* <SPI.h>    Arduino standard library
