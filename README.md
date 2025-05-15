# Project Documentation: PoseTrack - ESP32-Based Wearable Posture Tracking and Correction System

**Version:** 1.2
**Date:** May 10, 2025
**Author/Team:** PoseTrack Team

## 1. Abstract

This document provides a comprehensive overview of the PoseTrack system, an ESP32-based wearable device engineered for real-time posture monitoring and haptic feedback. PoseTrack employs multiple MPU6050 Inertial Measurement Units (IMUs) to meticulously track the user's spinal orientation, identifying deviations from a user-calibrated "ideal" posture. The system is designed to deliver localized haptic alerts via vibration motors upon detection of undesirable postural states. Communication with external devices, such as smartphones, is achieved through Bluetooth Low Energy (BLE), enabling real-time data visualization, system configuration, and remote actuation of the feedback mechanisms. PoseTrack aims to offer an accessible, adaptable, and effective technological solution for promoting posture awareness and facilitating corrective habits.

## 2. Introduction

### 2.1. Background and Significance

The prevalence of musculoskeletal disorders and chronic pain associated with poor posture is a growing global health concern, significantly impacted by increasingly sedentary lifestyles and the ubiquitous use of digital screens. Maintaining correct posture is crucial for spinal health, efficient biomechanics, and overall well-being. Wearable technology presents a unique opportunity for continuous, unobtrusive posture monitoring, offering personalized feedback to empower users to proactively manage and improve their postural habits. PoseTrack addresses this need by providing a tangible tool for real-time posture assessment and intervention.

### 2.2. Problem Statement

The core challenge lies in developing a wearable system that is not only accurate in its multi-point spinal tracking capabilities but also user-friendly, configurable, and effective in delivering intuitive feedback. Many existing solutions may be limited by single-point tracking, lack of personalization, or obtrusive feedback mechanisms. PoseTrack aims to overcome these limitations by using a distributed sensor network along the spine and offering a customizable approach to posture correction.

### 2.3. Project Goals and Objectives

The primary goal of PoseTrack is to create a robust and user-centric wearable system for posture improvement. Specific objectives include:

* **Hardware Development:** To design and assemble a compact wearable device utilizing an ESP32 microcontroller and four MPU6050 IMU sensors for comprehensive spinal tracking.
* **Accurate Orientation Tracking:** To implement algorithms capable of accurately determining the pitch and roll angles for distinct segments of the user's spine, providing a detailed postural profile.
* **Personalized Calibration:** To develop an intuitive calibration routine that allows users to define their unique "correct" posture baseline, accommodating individual anatomical differences.
* **Deviation Analysis:** To continuously compute and report angular deviations from the calibrated baseline, quantifying the extent of postural misalignment.
* **Haptic Feedback System:** To integrate vibration motors that provide discreet, localized haptic feedback, currently triggered via BLE commands, with the future goal of automated triggering based on deviation thresholds.
* **BLE Connectivity:** To establish reliable BLE communication for bi-directional data flow: transmitting processed sensor data (deviations) to a client application and receiving commands (e.g., to initiate motor vibration or adjust settings).

## 3. System Architecture

### 3.1. Hardware Components

* **Microcontroller:** ESP32-WROOM-32 (or compatible ESP32 module).
    * **Rationale:** Selected for its powerful dual-core processing capabilities, ample GPIO pins, integrated Wi-Fi and Bluetooth Low Energy (BLE 5.0) transceivers, and extensive community support within the Arduino ecosystem, making it ideal for connected wearable applications.
* **Inertial Measurement Units (IMUs):** 4 x MPU6050 modules.
    * **Rationale:** The MPU6050 is a cost-effective, widely available IMU that integrates a 3-axis accelerometer and a 3-axis gyroscope along with a Digital Motion Processor (DMP). Its I2C interface and selectable addresses (with the AD0 pin) allow multiple units to be used on a limited number of I2C buses.
* **Vibration Motors:** 4 x miniature DC eccentric rotating mass (ERM) vibration motors.
    * **Rationale:** Chosen for their small form factor, low power consumption (relative to other haptic actuators), and ability to provide clear, localized tactile feedback.
* **Power Source:** A 3.7V Li-ion battery is used, managed by a TP4056 charger and protection module. This allows for rechargeable and portable operation suitable for a wearable device. For development, USB power to the ESP32 board can also be used.
* **Supporting Components:** Flexible connecting wires, (optional) custom PCB or breakout boards for sensor mounting and wiring management, pull-up resistors for I2C lines (often integrated into MPU6050 breakout modules, but external resistors might be needed depending on bus length and capacitance).

### 3.2. Hardware Setup and Interfacing

* **IMU Connectivity Strategy:** To avoid the need for an I2C multiplexer and leverage the ESP32's capabilities, the four MPU6050 sensors are distributed across the ESP32's two hardware I2C peripherals. This allows for simultaneous communication with pairs of sensors.
    * **I2C Bus 0 (Default `Wire` object):**
        * SDA: Connected to ESP32 GPIO 21.
        * SCL: Connected to ESP32 GPIO 22.
        * Hosts two MPU6050s:
            * Sensor A: AD0 pin connected to GND (I2C Address: `0x68`).
            * Sensor B: AD0 pin connected to VCC (I2C Address: `0x69`).
    * **I2C Bus 1 (Custom `TwoWire` object, `I2C_Bus1`):**
        * SDA: Connected to ESP32 GPIO 33.
        * SCL: Connected to ESP32 GPIO 32.
        * Hosts two MPU6050s:
            * Sensor C: AD0 pin connected to GND (I2C Address: `0x68`).
            * Sensor D: AD0 pin connected to VCC (I2C Address: `0x69`).
* All sensors share a common 3.3V power supply and ground (GND) connection with the ESP32.
* **Vibration Motor Connectivity:** Each motor is driven by a dedicated digital output pin on the ESP32.
    * Motor 1 (associated with Sensor 0/A): ESP32 GPIO 13.
    * Motor 2 (associated with Sensor 1/B): ESP32 GPIO 12.
    * Motor 3 (associated with Sensor 2/C): ESP32 GPIO 14.
    * Motor 4 (associated with Sensor 3/D): ESP32 GPIO 27.
    * **Note:** Standard vibration motors can draw more current than an ESP32 GPIO pin can safely supply directly. Each motor is driven using a logic-level SMD MOSFET, with a gate resistor connected to the ESP32 GPIO, and a common rectifier diode (e.g., 1N400x series or a Schottky diode for lower voltage drop) connected in parallel with the motor (as a flyback diode) to protect the MOSFET from inductive voltage spikes.

### 3.3. Software Architecture and Design

* **Development Framework:** Arduino Core for ESP32, enabling rapid prototyping and access to a wide range of libraries.
* **Key Libraries Utilized:**
    * `Wire.h`: Standard Arduino library for I2C communication.
    * `Adafruit_MPU6050.h` & `Adafruit_Sensor.h`: Provide a high-level interface for configuring and reading data from the MPU6050 sensors.
    * `BLEDevice.h`, `BLEUtils.h`, `BLEServer.h`, `BLE2902.h`: ESP-IDF BLE libraries wrapped for Arduino, enabling the ESP32 to function as a BLE server, define services, characteristics, and handle client interactions.
    * `cmath` (for `atan2`, `sqrt`, `pow`, `M_PI`), `cstdlib` (for `atoi`).
* **Primary Data Structure (`SensorInfo` struct):** This struct is pivotal for organizing and managing the state and data associated with each of the four MPU6050 sensors. It includes:
    * `name`: A human-readable identifier (e.g., "Shoulder_Left").
    * `i2c_bus`: A pointer to the `TwoWire` instance (`I2C_Bus0` or `I2C_Bus1`) the sensor is connected to.
    * `address`: The unique I2C address of the sensor on its bus.
    * `mpu`: A pointer to the dynamically allocated `Adafruit_MPU6050` object for that sensor.
    * `initialized`: A boolean flag indicating successful sensor initialization.
    * `pitch`, `roll`: Floating-point values representing the current calculated orientation in degrees.
    * `baseline_pitch`, `baseline_roll`: Floating-point values storing the orientation captured during calibration.
    * `last_read_time`: An `unsigned long` storing the timestamp (in microseconds) of the last sensor read, crucial for calculating the time delta (`dt`) in the complementary filter.
* **Core Functional Modules (Software Blocks):**
    * **Sensor Initialization (`initializeSensor` function):** Responsible for dynamically creating the `Adafruit_MPU6050` object for a given sensor, attempting to establish communication via its assigned I2C bus and address, and configuring sensor parameters (accelerometer range, gyroscope range, filter bandwidth).
    * **Data Processing (`processSensorData` function):** This function is called repeatedly for each initialized sensor. It fetches raw accelerometer and gyroscope readings, calculates the time elapsed since the last reading (`dt`), and applies the complementary filter algorithm to fuse the data and produce updated pitch and roll angle estimates.
    * **Calibration (`performCalibration` function):** Orchestrates the user-guided calibration process. It collects multiple orientation samples from all sensors while the user maintains their target posture and then averages these samples to establish the `baseline_pitch` and `baseline_roll` for each sensor.
    * **BLE Communication (`setupBLE` function and Callback Classes):** This module initializes the ESP32 as a BLE GATT server. It defines a primary service and multiple characteristics: four for transmitting sensor deviation data (supporting READ and NOTIFY properties) and one for receiving motor control commands (supporting WRITE property). Callback classes (`MyServerCallbacks`, `MotorControlCallbacks`) are used to handle BLE events such as client connections/disconnections and incoming data writes.
    * **Motor Control (`activateMotor` function):** This utility function provides a simple interface to activate a specified vibration motor for a defined duration by controlling its corresponding GPIO pin.

## 4. Core Functionality Explained

### 4.1. Multi-Point Sensor Data Acquisition

The system architecture with four IMUs allows for a more granular understanding of spinal posture compared to single-sensor systems. Each MPU6050 is periodically polled for its 3-axis acceleration and 3-axis angular velocity data. The use of two separate I2C buses allows for potentially faster data acquisition from pairs of sensors, though the current implementation polls them sequentially within the main loop.

### 4.2. Orientation Calculation: The Complementary Filter

The orientation (pitch and roll) of each sensor relative to a fixed reference frame (gravity) is calculated using a complementary filter.
* **Pitch:** Typically represents forward/backward tilt.
* **Roll:** Typically represents side-to-side tilt.
* **Rationale for Complementary Filter:**
    * **Accelerometer-derived angles:** Calculated using `atan2` based on the gravity vector components. These are accurate for static or slow-moving conditions but are heavily affected by linear accelerations (e.g., walking, quick movements), leading to noisy readings.
    * **Gyroscope-derived angles:** Calculated by integrating the angular velocity readings over time (`gyro_rate * dt`). Gyroscopes provide smooth and responsive tracking of changes in orientation and are immune to linear accelerations. However, integration leads to accumulated error (drift) over time, causing the angle estimate to slowly diverge from the true orientation.
    * **Fusion:** The complementary filter combines the strengths of both sensors. It primarily relies on the integrated gyroscope data for high-frequency changes (fast movements) and uses the accelerometer data to correct for low-frequency drift (slow changes and maintaining an absolute reference to gravity). The `ALPHA` constant determines the cutoff frequency of this filter.
        `angle = ALPHA * (previous_angle + gyro_angular_change_since_last_reading) + (1.0 - ALPHA) * accelerometer_derived_angle`

### 4.3. User-Centric Calibration

Calibration is a critical step for personalization. Since "correct" posture varies between individuals and sensor placement might not be perfectly consistent, the `performCalibration` routine allows the system to learn the user's specific target posture.
1.  **Trigger:** Initiated by sending a 'C' character via the Serial Monitor.
2.  **User Action:** The user is instructed to assume and hold their ideal, upright posture.
3.  **Data Sampling:** The system collects `CALIBRATION_READINGS` (e.g., 100) sets of pitch and roll values from each active sensor.
4.  **Baseline Calculation:** For each sensor, the collected pitch and roll values are averaged. These averages become the `baseline_pitch` and `baseline_roll`.
5.  **State Update:** The `is_calibrated` flag is set, enabling deviation tracking.

### 4.4. Posture Deviation Analysis

Once calibrated, PoseTrack continuously monitors the current orientation of each sensor and compares it to its established baseline.
`pitch_deviation = current_sensor_pitch - sensor_baseline_pitch`
`roll_deviation = current_sensor_roll - sensor_baseline_roll`
These signed deviation values indicate both the magnitude and direction of misalignment (e.g., a positive pitch deviation might indicate slouching forward). This data is then made available via BLE and Serial output.

### 4.5. Bluetooth Low Energy (BLE) Interface

The ESP32 operates as a BLE GATT (Generic Attribute Profile) server, enabling wireless communication with a client (e.g., smartphone app).
* **Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b` (a unique identifier for the PoseTrack service).
* **Sensor Data Characteristics (Properties: Read, Notify):**
    * Four distinct characteristics, one for each MPU6050 sensor, are defined with the following UUIDs:
        * Sensor 0: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
        * Sensor 1: `c275f5e1-829b-45c7-b52f-6079d6a9b8b5`
        * Sensor 2: `a31e5518-685a-4f8f-90de-1f92a2b658f9`
        * Sensor 3: `928950e9-1a74-4396-8a73-01295a11a2a2`
    * **Data Format:** Each characteristic transmits data as a UTF-8 string in the format: "PitchDeviation,RollDeviation" (e.g., "10.1,-5.3"), where values are in degrees.
    * **Notifications:** When a BLE client subscribes to these characteristics, the ESP32 periodically sends updates (notifications) containing the latest deviation data. This allows for real-time data streaming without constant polling by the client.
* **Motor Control Characteristic (Property: Write):**
    * UUID: `3c761059-0073-46d2-b852-4579899f678c`
    * **Functionality:** Allows a connected BLE client to send a command to the ESP32. The firmware expects a string value: "1", "2", "3", or "4".
    * **Action:** Upon receiving a valid command, the ESP32 triggers the corresponding vibration motor.
* **Connection Management:** The `MyServerCallbacks` class monitors BLE connection events. If a client disconnects, the ESP32 automatically restarts BLE advertising to allow for reconnection or connection by a new client.

### 4.6. Haptic Feedback Mechanism

The `activateMotor` function provides the interface to the physical feedback system. When the `MotorControlCallbacks` class receives a valid numeric string ("1" through "4") on the motor control characteristic, it calls `activateMotor` with the corresponding motor index (0-3) and a fixed duration of 200ms. This results in a short, noticeable vibration at the location of the specified motor.

## 5. Software Implementation Details

### 5.1. `SensorInfo` Structure: The Core Data Organizer

The `SensorInfo` struct is fundamental. By creating an array of these structs (`SensorInfo sensors[4]`), the code can manage each sensor's unique properties (I2C bus, address, MPU object) and state (calibration data, current orientation) in an organized manner. This approach simplifies iterating through sensors for initialization, data processing, and reporting.

### 5.2. Overview of Key Functions

* **`setup()`:** This is the primary initialization routine executed once at startup. It performs:
    1.  Serial communication initialization (for debugging and calibration trigger).
    2.  Configuration of motor GPIO pins as outputs and ensuring they are initially OFF.
    3.  Initialization of the two I2C bus peripherals (`I2C_Bus0`, `I2C_Bus1`) with their respective SDA/SCL pins and communication speed (400kHz).
    4.  Assignment of specific configurations (name, I2C bus pointer, I2C address) to each element in the `sensors` array.
    5.  Iterative initialization of each MPU6050 sensor by calling `initializeSensor()`.
    6.  Setup and initiation of all BLE services and characteristics by calling `setupBLE()`.
* **`loop()`:** This function runs repeatedly after `setup()` completes. Its main responsibilities are:
    1.  Checking for incoming serial data to trigger the calibration routine (`performCalibration()`).
    2.  Continuously calling `processSensorData()` for each initialized sensor to update its orientation data.
    3.  If the system is calibrated (`is_calibrated == true`):
        * Periodically (controlled by `print_count` and `PRINT_INTERVAL`), calculate pitch and roll deviations for each sensor.
        * Print these deviations to the Serial Monitor.
        * If a BLE client is connected, format the deviation data as a string and send it via BLE notification for each sensor's characteristic.
    4.  Managing BLE connection state: If a device disconnects, it triggers the restarting of BLE advertising.
* **`initializeSensor(SensorInfo* sensor)`:** This function encapsulates the logic for setting up a single MPU6050. It dynamically allocates an `Adafruit_MPU6050` object, attempts to `begin` communication on the sensor's assigned I2C bus and address, and configures the sensor's operating parameters (accelerometer full-scale range, gyroscope full-scale range, and internal digital low-pass filter bandwidth).
* **`processSensorData(SensorInfo* sensor)`:** The heart of the orientation calculation. It reads raw data, computes `dt`, and applies the complementary filter equations to update the `sensor->pitch` and `sensor->roll` values.
* **`performCalibration(void)`:** Guides the user through the calibration process, collects sensor data, computes average baseline angles, and stores them in the respective `SensorInfo` structs.
* **`setupBLE(void)`:** Configures the ESP32 to act as a BLE GATT server. This includes initializing the BLE stack, creating a server instance, defining a service with a unique UUID, and then adding the data and control characteristics to this service, each with their specific UUIDs and properties (Read, Write, Notify). It also sets up the necessary callbacks for server and characteristic events.
* **`activateMotor(int motorIndex, int durationMs)`:** A utility function that directly controls a motor's GPIO pin to produce a vibration pulse of a specified duration.
* **`MyServerCallbacks` (Class):** Implements `onConnect` and `onDisconnect` methods that are invoked by the BLE stack when a client connects to or disconnects from the ESP32 server, updating the `deviceConnected` flag.
* **`MotorControlCallbacks` (Class):** Implements the `onWrite` method, which is invoked when a BLE client writes data to the motor control characteristic. This method parses the incoming command and triggers the appropriate motor.

## 6. Usage Instructions

### 6.1. Hardware Assembly

1.  **IMU Connections:** Carefully connect the four MPU6050 sensors to the ESP32, adhering to the I2C bus assignments (Bus 0: GPIO 21/22; Bus 1: GPIO 33/32) and I2C address settings (AD0 pin to GND for `0x68`, AD0 to VCC for `0x69` on each bus). Double-check wiring to prevent shorts or incorrect connections.
2.  **Motor Connections:** Connect the four vibration motors to their designated ESP32 GPIO pins (13, 12, 14, 27). Crucially, use appropriate driver circuits (logic-level SMD MOSFET with gate resistor and flyback diode) for each motor to protect the ESP32's GPIO pins from overcurrent. Ensure a common ground between the motor circuit and the ESP32.
3.  **Power:** Connect the 3.7V Li-ion battery via the TP4056 module to the ESP32's power input. Alternatively, use USB power for development.

### 6.2. Firmware Deployment

1.  **Environment:** Open the `.ino` sketch file in the Arduino IDE (with ESP32 core installed) or a PlatformIO project.
2.  **Board Configuration:** In the IDE/PlatformIO, select the correct ESP32 board type (e.g., "ESP32 Dev Module") and the COM port to which the ESP32 is connected.
3.  **Library Dependencies:** Ensure the following libraries are installed:
    * Adafruit MPU6050 (by Adafruit)
    * Adafruit Unified Sensor (by Adafruit, a dependency for the MPU6050 library)
    * The ESP32 BLE libraries are part of the ESP32 Arduino core and do not require separate installation.
4.  **Compilation and Upload:** Compile the sketch and upload the firmware to the ESP32.

### 6.3. System Operation and Testing

1.  **Serial Monitoring:** Open the Arduino IDE's Serial Monitor (or any serial terminal program) and set the baud rate to 115200. This will display initialization logs, status messages, calibration prompts, and sensor deviation data.
2.  **Calibration Procedure:**
    * Upon startup, the system will prompt for calibration, or you can trigger it at any time by sending the character 'C' (uppercase) or 'c' (lowercase) via the Serial Monitor input.
    * When prompted, adopt your desired "straight" or "ideal" posture and remain as still as possible for several seconds while the system collects data from all sensors.
    * The Serial Monitor will display confirmation once calibration is complete, along with the calculated baseline pitch and roll values for each sensor.
3.  **BLE Client Connection:**
    * On a smartphone or computer with BLE capabilities, install and open a generic BLE scanner application (e.g., "nRF Connect for Mobile" by Nordic Semiconductor, "LightBlue Explorer," or similar).
    * Use the scanner app to search for nearby BLE devices. Locate and connect to the device named "ESP32 Posture Tracker".
    * Once connected, the app should allow you to explore the services and characteristics offered by the ESP32.
    * Find the PoseTrack service (UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`).
    * Within this service, locate the four sensor data characteristics (refer to UUIDs in Section 4.5). For each of these, enable notifications (this might be labeled "Subscribe," "Listen," or an icon with downward arrows). You should then start receiving periodic updates of pitch and roll deviation data.
4.  **Motor Control Test (Haptic Feedback):**
    * In the BLE scanner app, while connected to the ESP32, find the Motor Control Characteristic (UUID: `3c761059-0073-46d2-b852-4579899f678c`).
    * Use the app's "write value" functionality for this characteristic. Ensure the data type for writing is set to "Text" or "UTF-8 String."
    * Write one of the following string values: "1", "2", "3", or "4".
    * Upon sending the command, the corresponding vibration motor on the PoseTrack device should activate for 200 milliseconds. Observe the Serial Monitor for confirmation messages from the ESP32.

## 7. System Limitations and Considerations

* **Motor Driver Circuitry:** While specified, the exact implementation details (specific MOSFET part number, resistor values) of the motor driver circuits are crucial for performance and safety and should be carefully chosen.
* **Blocking Motor Delay:** The `activateMotor` function uses a `delay()`, which blocks other operations during the vibration. For more responsive systems or complex feedback patterns, non-blocking techniques (e.g., using `millis()` or timers) would be preferable.
* **Complementary Filter Tuning:** The `ALPHA` value (0.98) is a common default. Optimal performance might require tuning this value based on the specific application and sensor noise characteristics.
* **Axis Mapping:** The calculation of pitch and roll from accelerometer/gyroscope axes (accel.acceleration.x, gyro.gyro.y, etc.) is dependent on the physical orientation of each MPU6050 sensor on the wearable device. The current mapping may need adjustment if sensors are mounted differently.
* **Power Consumption:** Continuous sensor polling and BLE communication can be power-intensive. For battery-powered wearable use, significant power optimization strategies (e.g., ESP32 deep/light sleep modes, adjusting polling rates, optimizing BLE connection parameters) would be necessary. The TP4056 module provides charging and basic protection but does not inherently optimize the ESP32's consumption.
* **Environmental Factors:** IMU readings can be affected by strong magnetic fields (though MPU6050 primarily uses accel/gyro) or extreme temperature changes.
* **Single-User Calibration:** The current system stores one set of calibration data. Multi-user support or dynamic recalibration features are not yet implemented.

## 8. Future Enhancements and Potential Improvements

* **On-Device Slouch Detection & Automated Feedback:**
    * Implement algorithms on the ESP32 to analyze `pitch_deviation` and `roll_deviation` against user-configurable thresholds.
    * Automatically trigger specific vibration motors when slouching is detected for a certain duration, providing immediate corrective feedback without BLE intervention.
* **Advanced Posture Analysis:**
    * Develop more sophisticated algorithms that interpret data from all four sensors collectively to identify complex postural patterns (e.g., spinal torsion, uneven shoulder height) beyond simple sagittal or coronal plane deviations.
* **Non-Blocking Haptic Engine:**
    * Refactor `activateMotor` and related logic to use non-blocking techniques (e.g., `millis()`-based timers or FreeRTOS tasks) to allow for complex vibration patterns (e.g., pulses, ramps) without halting the main processing loop.
* **BLE Configuration Service:**
    * Add new BLE characteristics to allow remote configuration of parameters such as slouch detection thresholds, vibration intensity/duration, `ALPHA` filter constant, or sensor sampling rates.
* **Power Management Strategies:**
    * Investigate and implement ESP32's light-sleep or deep-sleep modes, potentially waking up periodically for sensor reads or on interrupt from an accelerometer (if configured for motion detection).
    * Optimize BLE connection intervals and advertising parameters for reduced power draw.
* **Dedicated Mobile Application:**
    * Develop a custom iOS/Android application for a richer user experience, including:
        * Intuitive calibration guidance.
        * Real-time visualization of posture data (e.g., a spinal avatar).
        * Historical data logging and trend analysis.
        * Customizable feedback settings and reminders.
* **Enhanced Error Handling and System Stability:**
    * Implement more robust error checking for I2C communication (e.g., detecting sensor disconnects during operation).
    * Add watchdog timers to ensure system recovery from unexpected states.
* **Alternative Calibration Triggers:**
    * Integrate a physical button on the wearable device to initiate the calibration sequence.
    * Add a dedicated BLE characteristic to trigger calibration remotely from a connected app.
* **Multi-User Profiles & Data Storage:**
    * Implement functionality to store and switch between multiple user calibration profiles, potentially using the ESP32's non-volatile storage (NVS) or an external EEPROM/flash memory.
* **Machine Learning for Posture Classification:**
    * Explore collecting datasets of sensor readings corresponding to various postures and training a machine learning model (e.g., using TensorFlow Lite for Microcontrollers) for more nuanced posture recognition and classification directly on the ESP32.

## 9. Conclusion

The PoseTrack project successfully realizes a functional prototype of a multi-sensor, ESP32-based wearable system for posture tracking and haptic feedback. By effectively utilizing the ESP32's dual I2C capabilities for interfacing with four MPU6050 IMUs and leveraging Bluetooth Low Energy for data communication and remote control, the system provides a versatile and extensible platform. The implementation of a user-centric calibration routine and the clear reporting of postural deviations offer valuable insights for users. While the current version provides a solid foundation, the outlined future enhancements point towards a pathway for developing an even more sophisticated, autonomous, and user-friendly posture correction device. PoseTrack demonstrates the potential of integrated embedded systems in creating accessible and impactful personal health technology.
