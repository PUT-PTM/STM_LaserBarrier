# STM_LaserBarrier

## Overview
A laser barrier utilising a simple light sensor and a laser pointer to measure the speed of an object. Implemented on STM32 (STM32F407VG).
## Description
The board measures a speed of an object passing between a set of mirrors which reflect a single laser beam into a light a sensor. It measures the time elapsed between passing the first and the second laser beam and calculates the speed.

### How to use
-The user button starts the measurment and resets the variables if needed.
-Potentiometer 1 is used for LCD contrast
-Potentiometer 2 is used to calibrate the sensor to adjust for variable light levels in the enviroment

-Prepare the mirrors, a laser source and aim it at the sensor
-The first screen shows the "Ready to start message", beneath is the light sensor readout(left) and the calibration readout(right)
-Press the user button to begin
-On the top is the time readout in ms
-On the bottom is the speed readout in m/s
-On the right are indicators that tell you if the object crossed once or twice

Project components:
- STM32 microcontroller (STM32F407VG or equivalent)
![STM](readme/stm1.jpg?raw=true "STM")
- 2x16 lcd display (LCD-03336)
![LCD](readme/lcd1.jpg?raw=true "LCD")
- fotoresistor (20-30 kohm)
![FOT](readme/fot1.jpg?raw=true "FOT")
- 2x potentiometers
![POT](readme/pot1.jpg?raw=true "POT")


## Tools
- CoIDE v1.7.8
- additional appropriate compilers and/or drivers for your board
## How to run
Connect all the components.  
Pin layout:
![https://stm32f4-discovery.net/2014/06/library-16-interfacing-hd44780-lcd-controller-with-stm32f4/](readme/lcd.png?raw=true "LCD")

		
STM32 | fotoresistor
---|---
PA3	|	out	


STM32 | potentiometers
---|---
PC1	|	ADC1
V0(LCD)	|	ADC2

## How to compile
Compile using CoIDE and GCC compiler and run on the board.
## Future improvements
Better sensor, more robust and advanced object detection algorithm. 
## Attributions
Libraries for ADC, DELAY, GPIO and HD44780 can be found over [here](https://stm32f4-discovery.net/)
## License
Code licensed under the The MIT License.



The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.
Supervisor: Adam Bondyra