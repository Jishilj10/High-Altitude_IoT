# High Altitude platform based IoT
This project involves the development of a communication system between LowTier devices (sensors) and a GroundStation using LoRa (Long Range) communication technology.  The High altitude devices consist of sensor nodes equipped with Sony Spresence MCU with various sensors, such as MPU6050, BME680,  DHT fixed to an UAV. The GroundStation also acting as sensor node, receives data and beacon packets from the High altitude device.


## Introduction
The communication system is designed to collect data from various sensors on High altitude devices and transmit it to a GroundStation using LoRa communication. The LowTier devices periodically send beacon packets containing location information, and data packets with sensor readings are sent at regular intervals.

## Hardware Requirements
1. Sony Spresense MCUs
2. LoRa RFM95 communication modules
3. MPU6050 Sensor 
4. BME680 Sensor
5. DHT22 Sensor
6. Soil moisture sensors
7. SD card modules for data storage

## Libraries
The project utilizes the following Arduino libraries:
    1. **File.h**: Library for handling files on the SD card.
    2. **TinyMPU6050.h**: Library for MPU6050 sensor.
    3. **SDHCI.h**: Library for SD card communication.
    4. **LoRa.h**: Library for LoRa communication.
    5. **GNSS.h**: Library for Spresense GNSS.
    6. **Wire.h**: Library for I2C communication.
    7. **dht.h**: Library for DHT temperature and humidity sensor. \
Make sure to install these libraries before compiling the code.

## Configuration
1. Configure the LoRa module settings such as frequency, chip select, reset, and interrupt pins in both LowTier and GroundStation code.
2. Adjust the data transmission intervals and other parameters as needed.

## Setup
1. Connect the sensors (MPU6050, BME680, GNSS, DHT, gas, soil moisture) to the suitable devices.
2. Connect the LoRa modules to both the LowTier devices and the GroundStation.
3. Insert the SD card module for data storage in both the LowTier devices and the GroundStation.
4. Upload the corresponding code to each LowTier device and the GroundStation.

## Usage
1. Power on the LowTier devices and the GroundStation.
2. The LowTier devices will start sending beacon packets with location information.
3. Data packets with sensor readings will be sent periodically from the LowTier devices to the GroundStation.
4. The GroundStation receives and processes the packets, storing data in CSV files on the SD card.

## Data Format
The data format for beacon and data packets is specified in the code. Beacon packets contain information such as packet type, send time, source ID, latitude, longitude, and altitude. Data packets include packet type, data ID, send time, source ID, destination ID, length of data, data, and a counter.

## Communication Flow
1. **Beacon Packets**:
    - LowTier devices periodically send beacon packets with location information.
    - GroundStation receives and processes beacon packets.

2. **Data Packets**:
    - LowTier devices periodically send data packets with sensor readings.
    - GroundStation receives and processes data packets, storing data in CSV files.







