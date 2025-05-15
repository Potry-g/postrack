#include <Arduino.h>
#include <Wire.h>             // For I2C communication
#include <Adafruit_MPU6050.h> // Adafruit MPU6050 library
#include <Adafruit_Sensor.h>  // Dependency for Adafruit MPU6050 library
#include <cmath>              // For mathematical functions like atan2, sqrt, pow, M_PI
#include <cstdlib>            // For atoi (convert string to integer)

// --- BLE Libraries ---
#include <BLEDevice.h> // Core BLE functionalities
#include <BLEUtils.h>  // BLE utility functions
#include <BLEServer.h> // For creating a BLE server
#include <BLE2902.h>   // For BLE characteristic descriptors (e.g., for notifications)

// --- Hardware Pin Definitions ---
// I2C Bus 0 Pins
#define I2C0_SDA_PIN 21
#define I2C0_SCL_PIN 22

// I2C Bus 1 Pins
#define I2C1_SDA_PIN 33
#define I2C1_SCL_PIN 32

// MPU6050 I2C Addresses
#define MPU6050_ADDR_0 0x68 // Default MPU6050 address (AD0 pin LOW)
#define MPU6050_ADDR_1 0x69 // Alternative MPU6050 address (AD0 pin HIGH)

// Vibration Motor Pins
// IMPORTANT: Adjust these GPIO pins if your motors are connected differently.
#define MOTOR_PIN_1 13 // Motor for sensor 1 (e.g., Sholder_Right)
#define MOTOR_PIN_2 12 // Motor for sensor 2 (e.g., Spine_Lower)
#define MOTOR_PIN_3 18 // Motor for sensor 3 (e.g., Spine_Upper)
#define MOTOR_PIN_4 19 // Motor for sensor 4 (e.g., Sholder_Left)

// Array for easy indexed access to motor pins. Order should match sensor array.
const int motorPins[] = {MOTOR_PIN_1, MOTOR_PIN_2, MOTOR_PIN_3, MOTOR_PIN_4};

// --- Global Variables & Objects ---
// I2C Bus Instances
TwoWire I2C_Bus0 = TwoWire(0); // Hardware I2C peripheral 0
TwoWire I2C_Bus1 = TwoWire(1); // Hardware I2C peripheral 1

// Sensor Data Structure
struct SensorInfo
{
  String name;                     // Descriptive name (e.g., "Shoulder_Left")
  TwoWire *i2c_bus;                // Pointer to the I2C bus instance
  uint8_t address;                 // I2C address of the sensor
  Adafruit_MPU6050 *mpu = nullptr; // Pointer to the MPU6050 object, initialized to nullptr
  bool initialized = false;        // Tracks successful initialization

  float pitch = 0.0f; // Current pitch angle (degrees)
  float roll = 0.0f;  // Current roll angle (degrees)

  float baseline_pitch = 0.0f; // Calibrated baseline pitch
  float baseline_roll = 0.0f;  // Calibrated baseline roll

  unsigned long last_read_time = 0; // Timestamp for complementary filter's dt calculation
};

SensorInfo sensors[4];                                       // Array to store data for all 4 sensors
const int numSensors = sizeof(sensors) / sizeof(sensors[0]); // Should be 4

// Complementary Filter Configuration
const float ALPHA = 0.98f; // Weight for gyroscope data (accelerometer gets 1.0 - ALPHA)

// Calibration Configuration
const int CALIBRATION_READINGS = 100; // Number of samples to average during calibration
bool is_calibrated = false;           // Flag indicating if calibration has been performed

// Output Control (Serial Print & BLE Notification Frequency)
int print_count = 0;            // Loop counter for periodic data output
const int PRINT_INTERVAL = 300; // Output data every N loop cycles

// BLE Objects and Flags
BLEServer *pServer = nullptr;
BLEService *pService = nullptr;
BLECharacteristic *pCharacteristicSensor[numSensors]; // Characteristics for sensor data
BLECharacteristic *pControlCharacteristic = nullptr;  // Unified characteristic for motor and calibration commands
bool deviceConnected = false;
bool oldDeviceConnected = false;

// BLE UUIDs (Universally Unique Identifiers)
// Generate your own unique UUIDs from a site like https://www.uuidgenerator.net/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // Main service UUID

// Custom UUID for the unified Control Characteristic (motor and calibration)
#define CONTROL_CHARACTERISTIC_UUID "3c761059-0073-46d2-b852-4579899f678c" // Existing motor UUID, now repurposed

const char *SENSOR_DATA_CHARACTERISTIC_UUIDS[numSensors] = {
    "beb5483e-36e1-4688-b7f5-ea07361b26a8", // Sensor 0
    "c275f5e1-829b-45c7-b52f-6079d6a9b8b5", // Sensor 1
    "a31e5518-685a-4f8f-90de-1f92a2b658f9", // Sensor 2
    "928950e9-1a74-4396-8a73-01295a11a2a2"  // Sensor 3
};

// --- Function Declarations (Prototypes) ---
void activateMotor(int motorIndex, int durationMs);
bool initializeSensor(SensorInfo *sensor);
void processSensorData(SensorInfo *sensor);
void performCalibration();
void setupBLE();

// --- Motor Control Function ---
/**
 * @brief Activates a specific motor for a given duration.
 * @param motorIndex The index (0-3) of the motor to activate.
 * @param durationMs The duration (in milliseconds) to keep the motor ON.
 */
void activateMotor(int motorIndex, int durationMs)
{
  if (motorIndex < 0 || motorIndex >= numSensors)
  {
    Serial.print("Error: Invalid motor index received: ");
    Serial.println(motorIndex + 1);
    return;
  }

  int motorPin = motorPins[motorIndex];
  Serial.print("Motor ");
  Serial.print(motorIndex + 1);
  Serial.print(" (Pin: ");
  Serial.print(motorPin);
  Serial.print("): Activating for ");
  Serial.print(durationMs);
  Serial.println("ms");

  digitalWrite(motorPin, HIGH);
  delay(durationMs);
  digitalWrite(motorPin, LOW);

  Serial.print("Motor ");
  Serial.print(motorIndex + 1);
  Serial.println(": Deactivated.");
}

// --- BLE Callback Classes ---
/**
 * @brief Handles BLE server events like client connect/disconnect.
 */
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pSrv)
  {
    deviceConnected = true;
    Serial.println("BLE Client Connected");
  }

  void onDisconnect(BLEServer *pSrv)
  {
    deviceConnected = false;
    Serial.println("BLE Client Disconnected. Advertising will restart.");
  }
};

/**
 * @brief Handles write events to the unified control BLE characteristic (motor and calibration).
 */
class ControlCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pChar)
  {
    std::string rxValueStdStr = pChar->getValue();
    String rxValue = String(rxValueStdStr.c_str()); // Convert to Arduino String for easier comparison

    if (rxValue.length() > 0)
    {
      Serial.print("BLE Control Rx: '");
      Serial.print(rxValue);
      Serial.println("'");

      // Check for calibration command first
      if (rxValue.equalsIgnoreCase("c"))
      { // Case-insensitive check for 'c'
        Serial.println("Calibration triggered via BLE.");
        performCalibration();
      }
      // Else, try to parse as motor command
      else
      {
        int motorCommand = rxValue.toInt(); // Arduino String toInt() is robust

        if (motorCommand >= 1 && motorCommand <= numSensors)
        {
          activateMotor(motorCommand - 1, 2000); // Convert 1-4 to 0-3 index
        }
        else
        {
          // If toInt() failed (returned 0 for non-numeric) or was out of range
          Serial.print("Error: Invalid command or motor number '");
          Serial.print(rxValue);
          Serial.println("'");
        }
      }
    }
  }
};

// --- Sensor Initialization and Processing Functions ---
/**
 * @brief Initializes a single MPU6050 sensor.
 * @param sensor Pointer to the SensorInfo struct for the sensor.
 * @return True if initialization was successful, false otherwise.
 */
bool initializeSensor(SensorInfo *sensor)
{
  Serial.print("Initializing ");
  Serial.print(sensor->name);
  Serial.print(" (Addr: 0x");
  Serial.print(sensor->address, HEX);
  Serial.print(", Bus: ");
  Serial.print(sensor->i2c_bus == &I2C_Bus0 ? "0" : "1");
  Serial.print(")...");

  sensor->mpu = new Adafruit_MPU6050();

  if (!sensor->mpu->begin(sensor->address, sensor->i2c_bus))
  {
    Serial.println(" Failed!");
    sensor->initialized = false;
    delete sensor->mpu;
    sensor->mpu = nullptr;
    return false;
  }

  Serial.println(" OK!");
  sensor->initialized = true;
  sensor->mpu->setAccelerometerRange(MPU6050_RANGE_8_G);
  sensor->mpu->setGyroRange(MPU6050_RANGE_500_DEG);
  sensor->mpu->setFilterBandwidth(MPU6050_BAND_21_HZ);
  sensor->last_read_time = micros();
  return true;
}

/**
 * @brief Reads sensor data and updates pitch/roll using a complementary filter.
 * @param sensor Pointer to the SensorInfo struct for the sensor.
 */
void processSensorData(SensorInfo *sensor)
{
  if (!sensor->initialized || sensor->mpu == nullptr)
    return;

  sensors_event_t accel, gyro, temp;
  sensor->mpu->getAccelerometerSensor()->getEvent(&accel);
  sensor->mpu->getGyroSensor()->getEvent(&gyro);

  unsigned long now = micros();
  float dt = (float)(now - sensor->last_read_time) / 1000000.0f;
  sensor->last_read_time = now;

  float accel_pitch = atan2(accel.acceleration.x, sqrt(pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2))) * 180.0f / M_PI;
  float accel_roll = atan2(accel.acceleration.y, sqrt(pow(accel.acceleration.x, 2) + pow(accel.acceleration.z, 2))) * 180.0f / M_PI;

  float gyro_roll_change = gyro.gyro.x * (180.0f / M_PI) * dt;
  float gyro_pitch_change = gyro.gyro.y * (180.0f / M_PI) * dt;

  sensor->pitch = ALPHA * (sensor->pitch + gyro_pitch_change) + (1.0f - ALPHA) * accel_pitch;
  sensor->roll = ALPHA * (sensor->roll + gyro_roll_change) + (1.0f - ALPHA) * accel_roll;
}

/**
 * @brief Performs calibration by averaging sensor readings to set baseline pitch/roll.
 */
void performCalibration()
{
  Serial.println("\n--- Calibration Started ---");
  Serial.println("Hold still in desired 'straight' posture.");
  delay(2000);

  float total_pitch[numSensors] = {0.0f};
  float total_roll[numSensors] = {0.0f};
  int valid_readings[numSensors] = {0};

  Serial.print("Taking ");
  Serial.print(CALIBRATION_READINGS);
  Serial.println(" readings...");
  for (int i = 0; i < CALIBRATION_READINGS; i++)
  {
    for (int j = 0; j < numSensors; j++)
    {
      if (sensors[j].initialized)
      {
        processSensorData(&sensors[j]);
        total_pitch[j] += sensors[j].pitch;
        total_roll[j] += sensors[j].roll;
        valid_readings[j]++;
      }
    }
    delay(20);
    if ((i + 1) % (CALIBRATION_READINGS / 10) == 0)
      Serial.print(".");
  }
  Serial.println("\nReadings complete.");

  Serial.println("Calculating baselines:");
  for (int i = 0; i < numSensors; i++)
  {
    if (valid_readings[i] > 0)
    {
      sensors[i].baseline_pitch = total_pitch[i] / valid_readings[i];
      sensors[i].baseline_roll = total_roll[i] / valid_readings[i];
      Serial.print("  ");
      Serial.print(sensors[i].name);
      Serial.print(": Baseline Pitch=");
      Serial.print(sensors[i].baseline_pitch, 2);
      Serial.print(" deg, Roll=");
      Serial.print(sensors[i].baseline_roll, 2);
      Serial.println(" deg");
    }
    else
    {
      Serial.print("  Warning: No valid readings for ");
      Serial.print(sensors[i].name);
      Serial.println(". Calibration failed for this sensor.");
    }
  }

  is_calibrated = true;
  Serial.println("--- Calibration Complete ---");
  Serial.println("Posture tracking active. Deviations will be shown periodically.");
  print_count = PRINT_INTERVAL;
}

// --- BLE Setup Function ---
/**
 * @brief Initializes and starts BLE services and characteristics.
 */
void setupBLE()
{
  Serial.println("Setting up BLE...");
  BLEDevice::init("PoseTrack");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(SERVICE_UUID);

  // Create characteristics for sensor data
  for (int i = 0; i < numSensors; i++)
  {
    pCharacteristicSensor[i] = pService->createCharacteristic(
        SENSOR_DATA_CHARACTERISTIC_UUIDS[i],
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristicSensor[i]->addDescriptor(new BLE2902());
    pCharacteristicSensor[i]->setValue("0.0,0.0");
  }

  // Create the unified Control Characteristic (for motor and calibration)
  pControlCharacteristic = pService->createCharacteristic(
      CONTROL_CHARACTERISTIC_UUID, // Using the existing motor UUID
      BLECharacteristic::PROPERTY_WRITE);
  pControlCharacteristic->setCallbacks(new ControlCallbacks()); // Assign the updated callback

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising started. Waiting for client connection...");
}

// --- Arduino Setup Function ---
void setup()
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("\n--- PoseTrack ESP32 Initializing ---");

  // Initialize Motor Pins
  Serial.println("Initializing motor pins...");
  for (int i = 0; i < numSensors; i++)
  {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
    Serial.print("  Motor ");
    Serial.print(i + 1);
    Serial.print(" (Pin: ");
    Serial.print(motorPins[i]);
    Serial.println(") ready.");
  }

  // Initialize I2C Buses
  Serial.println("Initializing I2C buses...");
  I2C_Bus0.begin(I2C0_SDA_PIN, I2C0_SCL_PIN, 400000);
  Serial.println("  I2C Bus 0 (SDA:" + String(I2C0_SDA_PIN) + ", SCL:" + String(I2C0_SCL_PIN) + ") initialized.");
  I2C_Bus1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN, 400000);
  Serial.println("  I2C Bus 1 (SDA:" + String(I2C1_SDA_PIN) + ", SCL:" + String(I2C1_SCL_PIN) + ") initialized.");

  // Configure MPU6050 Sensors
  Serial.println("Configuring MPU6050 sensors...");
  sensors[0].name = "Shoulder_Left";
  sensors[0].i2c_bus = &I2C_Bus0;
  sensors[0].address = MPU6050_ADDR_0;
  sensors[1].name = "Spine_Upper";
  sensors[1].i2c_bus = &I2C_Bus0;
  sensors[1].address = MPU6050_ADDR_1;
  sensors[2].name = "Shoulder_Right";
  sensors[2].i2c_bus = &I2C_Bus1;
  sensors[2].address = MPU6050_ADDR_0;
  sensors[3].name = "Spine_Lower";
  sensors[3].i2c_bus = &I2C_Bus1;
  sensors[3].address = MPU6050_ADDR_1;

  // Initialize MPU6050 Sensors
  int initialized_count = 0;
  for (int i = 0; i < numSensors; i++)
  {
    if (initializeSensor(&sensors[i]))
      initialized_count++;
    delay(100);
  }

  Serial.print(initialized_count);
  Serial.print("/");
  Serial.print(numSensors);
  Serial.println(" sensors initialized.");

  if (initialized_count == 0)
  {
    Serial.println("!!! CRITICAL ERROR: NO SENSORS INITIALIZED. CHECK WIRING/ADDRESSES. HALTING. !!!");
    while (true)
      delay(1000);
  }
  else if (initialized_count < numSensors)
  {
    Serial.println("Warning: Not all sensors initialized. Functionality may be limited.");
  }

  setupBLE();

  Serial.println("\nSystem ready. Send 'C'/'c' via Serial or BLE (to Control Characteristic) for calibration.");
  Serial.println("----------------------------------------------------");
}

// --- Arduino Main Loop ---
void loop()
{
  // Check for calibration command from Serial input
  if (Serial.available())
  {
    char command = Serial.read();
    if (command == 'C' || command == 'c')
    {
      Serial.println("Calibration triggered via Serial.");
      performCalibration();
    }
  }

  // Continuously process data for all initialized sensors
  for (int i = 0; i < numSensors; i++)
  {
    if (sensors[i].initialized)
    {
      processSensorData(&sensors[i]);
    }
  }

  // If calibrated, periodically print deviations and send via BLE
  if (is_calibrated)
  {
    if (print_count >= PRINT_INTERVAL)
    {
      Serial.println("--- Posture Deviations (degrees) ---");
      for (int i = 0; i < numSensors; i++)
      {
        if (sensors[i].initialized)
        {
          float pitch_dev = sensors[i].pitch - sensors[i].baseline_pitch;
          float roll_dev = sensors[i].roll - sensors[i].baseline_roll;

          Serial.print("  ");
          Serial.print(sensors[i].name);
          Serial.print(": PitchDev=");
          Serial.print(pitch_dev, 1);
          Serial.print(", RollDev=");
          Serial.print(roll_dev, 1);
          Serial.println();

          if (deviceConnected)
          {
            String ble_data = String(pitch_dev, 1) + "," + String(roll_dev, 1);
            pCharacteristicSensor[i]->setValue(ble_data.c_str());
            pCharacteristicSensor[i]->notify();
          }
        }
      }
      Serial.println("------------------------------------");
      print_count = 0;
    }
    else
    {
      print_count++;
    }
  }
  else
  {
    if (print_count >= PRINT_INTERVAL)
    {
      Serial.println("System awaiting calibration ('C'/'c' command via Serial or BLE Control Characteristic)...");
      print_count = 0;
    }
    else
    {
      print_count++;
    }
  }

  // Handle BLE connection state changes (e.g., restart advertising on disconnect)
  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);
    pServer->startAdvertising();
    Serial.println("BLE Advertising Restarted.");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }

  delay(10);
}
