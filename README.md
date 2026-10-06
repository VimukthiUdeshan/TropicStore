# 🍌 TropicStore – Smart Fruit Ripening Chamber

## 📌 Project Overview

**TropicStore** is an IoT-based Smart Fruit Ripening Chamber designed to provide an automated and controlled environment for fruit ripening.

The system monitors important environmental and ripening-related parameters using sensors connected to an **ESP32 microcontroller**. Based on the collected sensor data and configured thresholds, the system automatically controls different actuators such as an exhaust fan, servo-operated ventilation door, mist maker, and lighting.

The system integrates **MQTT communication, AWS IoT, and a dedicated mobile application** to provide remote monitoring and control of the ripening chamber.

The main objective of TropicStore is to provide a more **controlled, automated, and efficient fruit ripening process** while reducing the need for continuous manual monitoring.

---

## 🎓 Academic Context

TropicStore was developed as part of the **Computer Engineering Project (CO3302)** at the **Department of Computer Engineering, Faculty of Engineering, University of Sri Jayewardenepura**.

The project combines concepts from:

* Embedded Systems
* Internet of Things (IoT)
* Cloud Computing
* Sensor Networks
* Automation and Control
* Mobile Application Development
* MQTT Communication
* AWS Cloud Services

---

## 🎯 Objectives

The main objectives of the TropicStore project are:

* 🌡️ Monitor temperature inside the ripening chamber.
* 💧 Monitor humidity levels.
* 🧪 Detect changes in gas concentration associated with fruit ripening.
* 🚪 Automatically control the ventilation door using a servo motor.
* 🌬️ Automatically control the exhaust fan.
* 💦 Control humidity using a mist maker.
* 💡 Provide controlled lighting inside the chamber.
* ☁️ Send sensor data to the cloud using MQTT and AWS IoT.
* 📱 Provide a dedicated mobile application for remote monitoring and control.
* ⚙️ Automate the ripening environment according to predefined thresholds.

---

## 🏗️ System Architecture

The TropicStore system consists of four main sections:

### 1. Sensing Layer

Sensors collect information about the internal conditions of the chamber.

The system uses sensors such as:

* **DHT22** – Temperature and humidity monitoring
* **MQ-3** – Gas concentration monitoring related to the ripening process

The sensor readings are processed by the ESP32.

### 2. Control Layer

The **ESP32** acts as the main controller of the system.

Based on sensor readings and configured threshold values, the ESP32 controls:

* Servo motor
* Exhaust fan
* Mist maker
* LED lighting
* Other connected actuators

### 3. Cloud / IoT Layer

The ESP32 communicates with the cloud using **MQTT** and **AWS IoT**.

This allows the system to:

* Publish sensor readings
* Monitor chamber conditions remotely
* Receive control commands
* Update configurable threshold values
* Connect the physical chamber with the mobile application

### 4. Mobile Application

A dedicated **mobile application** was developed as part of TropicStore.

The application provides a user interface for:

* Monitoring chamber conditions
* Viewing sensor readings
* Monitoring actuator states
* Controlling supported system functions
* Configuring system parameters
* Interacting with the IoT-enabled ripening chamber remotely

---

## 🏗️ High-Level System Architecture

```text
                         ┌──────────────────────┐
                         │     Mobile App       │
                         │                      │
                         │ Monitoring & Control │
                         └──────────┬───────────┘
                                    │
                                    │
                              MQTT / Cloud
                                    │
                                    ▼
                         ┌──────────────────────┐
                         │      AWS IoT         │
                         │                      │
                         │ Cloud Communication  │
                         └──────────┬───────────┘
                                    │
                                    │ MQTT
                                    ▼
                         ┌──────────────────────┐
                         │        ESP32         │
                         │                      │
                         │ Control & Processing │
                         └───────┬───────┬──────┘
                                 │       │
                    ┌────────────┘       └────────────┐
                    ▼                                 ▼
          ┌──────────────────┐              ┌──────────────────┐
          │     Sensors      │              │    Actuators     │
          │                  │              │                  │
          │ DHT22            │              │ Servo Motor      │
          │ MQ-3             │              │ Exhaust Fan      │
          │                  │              │ Mist Maker       │
          └──────────────────┘              │ LED              │
                                            └──────────────────┘
```

---

## 🔧 Hardware Components

| Component        | Purpose                                            |
| ---------------- | -------------------------------------------------- |
| ESP32            | Main microcontroller and IoT communication         |
| DHT22            | Temperature and humidity measurement               |
| MQ-3 Gas Sensor  | Gas concentration/ripening indication              |
| Servo Motor      | Controls the ventilation door                      |
| Exhaust Fan      | Removes excess gases/air from the chamber          |
| Mist Maker       | Increases humidity when required                   |
| LED              | Chamber lighting                                   |
| Relay Modules    | Controls high-power components                     |
| Buck Converter   | Provides regulated voltage for required components |
| Ripening Chamber | Controlled physical environment                    |

---

## 💻 Software & Technologies

The project uses the following technologies:

* **Arduino IDE**
* **C/C++**
* **ESP32**
* **MQTT**
* **AWS IoT Core**
* **AWS Cloud Services**
* **Mobile Application**
* **Arduino Libraries**
* **Git & GitHub**

---

## 📂 Repository Structure

```text
TropicStore/
│
├── TropicStore.ino
│
├── README.md
│
└── ...
```

### `TropicStore.ino`

The `.ino` file contains the main ESP32 firmware responsible for:

* Initializing sensors
* Reading sensor values
* Processing sensor data
* Applying threshold conditions
* Controlling actuators
* Controlling the servo-operated ventilation door
* Managing the exhaust fan
* Managing the mist maker
* Managing the LED
* Connecting to Wi-Fi
* Communicating through MQTT
* Communicating with AWS IoT
* Receiving remote commands
* Updating configurable threshold values

---

## 📱 Mobile Application

TropicStore includes a dedicated mobile application developed to provide users with an accessible interface for interacting with the smart ripening chamber.

The application communicates with the IoT infrastructure and allows users to remotely monitor and manage the system.

### Main functions include:

* 📊 Viewing real-time sensor information
* 🌡️ Monitoring temperature
* 💧 Monitoring humidity
* 🧪 Monitoring gas sensor readings
* ⚙️ Monitoring system and actuator status
* 🎛️ Controlling supported actuators
* 🔧 Configuring threshold values
* 📡 Interacting with the chamber remotely

The mobile application provides an additional layer between the user and the physical ripening chamber, making the system easier to monitor and operate.

---

## ⚙️ System Operation

The general operation of the system is as follows:

```text
        ┌─────────────────────┐
        │       Sensors       │
        │                     │
        │  DHT22 + MQ-3       │
        └──────────┬──────────┘
                   │
                   ▼
        ┌─────────────────────┐
        │        ESP32        │
        │                     │
        │ Sensor Processing   │
        │ Threshold Checking  │
        │ Control Logic       │
        └───────┬─────┬───────┘
                │     │
        ┌───────┘     └────────────┐
        ▼                          ▼
┌───────────────┐          ┌────────────────┐
│   Actuators   │          │   AWS IoT      │
│               │          │     MQTT       │
│ Servo         │          │                │
│ Fan           │          │ Cloud & Mobile │
│ Mist Maker    │          │ Communication  │
│ LED           │          └────────────────┘
└───────────────┘
```

---

## 🌡️ Environmental Monitoring

The DHT22 sensor continuously monitors:

### Temperature

Temperature is monitored to ensure that the chamber remains within the desired operating range.

### Humidity

Humidity is monitored to maintain suitable conditions for fruit ripening.

If the humidity falls below the configured threshold, the system can activate the mist maker.

---

## 🧪 Gas Monitoring

The MQ-3 sensor is used to monitor changes in gas concentration inside the chamber.

The ESP32 reads the analog output from the sensor and compares the value against a configured threshold.

When the gas level reaches the configured condition, the system can activate the ventilation system.

> **Note:** MQ-series gas sensors require proper calibration and their readings can be affected by environmental conditions. Therefore, threshold values should be calibrated experimentally for the specific chamber and fruit type.

---

## 🚪 Automatic Ventilation

TropicStore uses a **servo motor** to control the ventilation door.

The servo moves between predefined positions depending on the system state.

For example:

```text
Normal Condition
       │
       ▼
Ventilation Door CLOSED
       │
       │ Gas level increases
       ▼
Threshold Reached
       │
       ▼
Ventilation Door OPEN
       │
       ▼
Exhaust Fan Activated
       │
       ▼
Gas / Excess Air Removed
```

This allows accumulated gases and excess air to be removed from the chamber automatically.

---

## 💦 Humidity Control

The mist maker is used to increase the humidity inside the chamber.

The ESP32 compares the measured humidity with the configured threshold.

```text
Humidity < Threshold
        │
        ▼
Mist Maker ON
        │
        ▼
Humidity increases
        │
        ▼
Desired Humidity Reached
        │
        ▼
Mist Maker OFF
```

---

## 🌬️ Exhaust Fan Control

The exhaust fan is used for ventilation and gas removal.

The fan can be activated automatically when the gas concentration reaches the configured threshold.

The ventilation system works together with the servo-controlled door to improve airflow through the chamber.

---

## ☁️ AWS IoT & MQTT

TropicStore uses **MQTT** for communication between the ESP32 and the cloud.

The ESP32 can publish information such as:

* Temperature
* Humidity
* Gas sensor readings
* Actuator states
* System status

The cloud communication also allows control information to be sent back to the ESP32.

This creates a two-way communication system:

```text
                 AWS IoT
                /       \
               /         \
          MQTT             MQTT
           ▲                 ▼
           │                 │
           ▼                 ▼
        ESP32  ◄──────────► Mobile App
           │
           │
     ┌─────┴─────┐
     ▼           ▼
  Sensors     Actuators
```

---

## 🔄 Remote Threshold Control

One of the IoT features of TropicStore is the ability to configure system thresholds remotely.

Instead of changing values directly in the ESP32 firmware every time, threshold values can be sent through the MQTT/AWS IoT communication system.

This makes the system more flexible and allows operating conditions to be adjusted remotely through the mobile application.

---

## 🔌 Power System

Different components in the system require different voltage levels.

The system therefore uses regulated power conversion where necessary.

The ESP32 operates using its required supply voltage, while external components such as motors, fans, relays, and the mist maker are powered according to their individual electrical requirements.

> **Important:** High-current or high-power components should not be powered directly from ESP32 GPIO pins. Appropriate relay modules, external power supplies, and voltage regulators should be used.

---

## 🧠 Control Logic

The overall control logic can be summarized as:

```text
Start
  │
  ▼
Initialize ESP32
  │
  ▼
Connect to Wi-Fi
  │
  ▼
Connect to AWS IoT / MQTT
  │
  ▼
Read Sensors
  │
  ├── Temperature
  ├── Humidity
  └── Gas Level
  │
  ▼
Check Thresholds
  │
  ├── Humidity Low?
  │       └── YES → Mist Maker ON
  │
  ├── Gas Level High?
  │       └── YES → Servo Door OPEN
  │                    Exhaust Fan ON
  │
  ▼
Publish Sensor Data
  │
  ▼
Check Incoming MQTT Commands
  │
  ▼
Update Control Parameters
  │
  ▼
Repeat
```

---

## 🚀 Getting Started

### 1. Install Arduino IDE

Install the Arduino IDE on your computer.

### 2. Install ESP32 Board Support

Add ESP32 board support to Arduino IDE and select the appropriate ESP32 board.

### 3. Install Required Libraries

Install the libraries required by the project through the Arduino IDE Library Manager.

Depending on the final firmware configuration, these may include libraries for:

* DHT22
* Servo control
* MQTT
* Wi-Fi
* AWS IoT communication

### 4. Configure Wi-Fi

Update the firmware with the required Wi-Fi configuration.

```cpp
const char* WIFI_SSID = "######";
const char* WIFI_PASSWORD = "#######";
```

### 5. Configure AWS IoT

Configure the required AWS IoT endpoint, certificates, keys, and MQTT topics according to the project's AWS IoT setup.

**Do not upload private certificates, private keys, passwords, or other sensitive credentials to GitHub.**

### 6. Configure Sensor Thresholds

Configure the required threshold values according to the calibration results and operating requirements.

### 7. Connect the Hardware

Connect the sensors and actuators according to the project's ESP32 pin configuration.

### 8. Upload the Firmware

Connect the ESP32 to the computer through USB and upload the `.ino` firmware using Arduino IDE.

### 9. Configure the Mobile Application

Install and configure the TropicStore mobile application according to the application's setup requirements.

Ensure that the application is connected to the same IoT infrastructure and MQTT communication system used by the ESP32.

### 10. Monitor the System

Use the Arduino Serial Monitor and mobile application to observe:

* Sensor readings
* Wi-Fi connection status
* MQTT connection status
* AWS IoT connection
* Actuator states
* Threshold events
* System status
* Remote control operations

---

## 🧪 Calibration

Proper sensor calibration is important for reliable operation.

The MQ-3 sensor in particular should be allowed to stabilize and should be calibrated under the conditions in which the chamber will operate.

Threshold values should be determined experimentally rather than relying only on default values.

Recommended calibration process:

1. Power the sensor and allow it to stabilize.
2. Observe the sensor readings under normal chamber conditions.
3. Record readings over a period of time.
4. Introduce the relevant ripening conditions.
5. Observe how the sensor values change.
6. Determine suitable threshold values.
7. Test the actuator response.
8. Fine-tune the thresholds.

---

## 🔐 Security

AWS IoT credentials and private keys must be kept secure.

The following information should **never be committed to a public GitHub repository**:

* AWS private keys
* Device certificates
* Wi-Fi passwords
* AWS access keys
* MQTT credentials
* Other authentication secrets

For public repositories, sensitive credentials should be stored separately and loaded securely during deployment.

---

## 📊 System Features

The completed TropicStore system provides:

* ✅ Automated environmental monitoring
* ✅ Automated ventilation
* ✅ Automated humidity control
* ✅ Servo-controlled ventilation door
* ✅ Gas-level monitoring
* ✅ MQTT communication
* ✅ AWS IoT integration
* ✅ Remote threshold configuration
* ✅ Mobile application
* ✅ Remote monitoring and control
* ✅ Reduced manual monitoring

---

## 🔮 Future Improvements

Potential future improvements to TropicStore include:

### 1. 📱 Mobile Application Enhancements

Further improvements to the mobile application could include advanced dashboards, improved visualization, and enhanced user interaction.

### 2. 🤖 Machine-Learning-Based Ripeness Prediction

Machine learning could be integrated to analyze sensor data and predict the ripeness stage of fruits more accurately.

### 3. 🍌 Multi-Fruit Support

The system could be expanded to support different fruit types with optimized environmental conditions and threshold profiles for each fruit.

### 4. 📊 Advanced Cloud Analytics

Additional cloud-based analytics could be implemented to identify trends, generate reports, and provide insights into the ripening process.

### 5. 🌡️ Additional Environmental Sensors

Additional sensors could be integrated to monitor parameters such as CO₂, ethylene, light intensity, and air quality for more accurate ripening analysis.

### 6. 📷 Camera-Based Fruit Monitoring

A camera system could be added to visually analyze fruits and estimate ripeness using computer vision techniques.

### 7. ⚡ Improved Energy Efficiency

The hardware and control algorithms could be optimized to reduce energy consumption, particularly for high-power components.

---

## 👥 Project Information

**Project Name:** TropicStore – Smart Fruit Ripening Chamber

**Course:** Computer Engineering Project (CO3302)

**Department:** Department of Computer Engineering

**Faculty:** Faculty of Engineering

**University:** University of Sri Jayewardenepura

**Project Area:** IoT / Embedded Systems / Cloud Computing / Mobile Application Development

**Controller:** ESP32

**Cloud Platform:** AWS IoT

**Communication Protocol:** MQTT

---

## 📜 License

This project was developed as an academic engineering project under the **Computer Engineering Project (CO3302)** at the **Department of Computer Engineering, Faculty of Engineering, University of Sri Jayewardenepura**.

The source code and project materials are intended primarily for educational and academic purposes.

---

## ⭐ Acknowledgements

TropicStore combines embedded systems, IoT communication, cloud computing, sensor technology, automation, and mobile application development to create a smart and connected fruit ripening solution.

**TropicStore – Making Fruit Ripening Smarter. 🍌🌱**
