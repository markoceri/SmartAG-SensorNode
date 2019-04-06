# SmartAG SensorNode
SmartAG SensorNode is a funny project that catch data from field and send it to Sigfox Backend. SensorNode aims to implement a simple weather station with soil moisture sensor, leaf wetness sensors, soil temperature and other useful sensors for smart and precision farming.

This project is under active development, feel free to make improvements.

# Project Parts
## Board
Arduino MKR Fox 1200
https://store.arduino.cc/arduino-mkrfox1200

## Sensors
- DHT22 for Temperature and Humidity (Internal)
- AM2305 for Temperature and Humidity (External)
- DS1820 for soil Temperature
- Capacitive soil Moisture sensor
- Leaf wetness (used an analogic Rain Sensor)
- TODO: Add BMP280 Pressure sensor
- TODO: Add Anemometer

## Other components
- 2x 18650 battery (2p)
- 2x 6v Solar Panel
- 1x TP4056 Lithium Charger

## Libraries
- DHT Sensor Library
- DallasTemperature
- Arduino Low Power
- Sigfox MKR
- Statistics
- Battery
- TODO: BMP280

## Features
- Check battery level
- Check sensors errors and comunicate it in message bitfield
- Switch to Long sleep time when battery in to low
- Stop sleep by pulling down a pin (usefull for reprogramming)
- Optimized consuption powering off sensors after reading
- TODO: Parse response message
- TODO: Change deep sleep from downlink message


