# 🌱 Threeshold Farm — Smart Agriculture System

An **IoT-based Smart Farm system** built using **ESP32** to monitor environmental conditions, control farm devices, and provide real-time communication through **MQTT** and data storage using **Supabase**.

The system combines sensors, actuators, an LCD display, and cloud services to create a simple and scalable smart agriculture solution.

---

## 📌 Project Overview

**Threeshold Farm** is designed to automate and monitor basic agricultural operations using an ESP32 microcontroller.

The system can:

* 🌞 Monitor light intensity using an **LDR sensor**
* 🚨 Detect objects using an **IR sensor**
* ⚙️ Control a **Servo Motor**
* 🔊 Control a **Buzzer**
* 💡 Display system status using **Red & Green LEDs**
* 🖥️ Display messages on a **16×2 I2C LCD**
* 🔘 Provide manual control using a **Push Button**
* ☁️ Store sensor readings in **Supabase**
* 📡 Communicate with external systems using **MQTT over TLS**

---

## 🏗️ System Architecture

```text
                    ┌───────────────────┐
                    │      ESP32        │
                    │   Main Controller │
                    └─────────┬─────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
   ┌─────────┐          ┌─────────┐          ┌──────────┐
   │   LDR   │          │   IR    │          │  Button  │
   │ Sensor  │          │ Sensor  │          │          │
   └─────────┘          └─────────┘          └──────────┘
        │                     │                     │
        └─────────────────────┼─────────────────────┘
                              │
                              ▼
                    ┌───────────────────┐
                    │   ESP32 Logic     │
                    └─────────┬─────────┘
                              │
             ┌────────────────┼────────────────┐
             │                │                │
             ▼                ▼                ▼
        ┌─────────┐      ┌─────────┐      ┌─────────┐
        │  Servo  │      │ Buzzer  │      │  LEDs   │
        └─────────┘      └─────────┘      └─────────┘
                             
                              │
                ┌─────────────┴─────────────┐
                │                           │
                ▼                           ▼
        ┌──────────────┐             ┌──────────────┐
        │    MQTT      │             │   Supabase   │
        │   HiveMQ     │             │   Database   │
        └──────────────┘             └──────────────┘
```

---

## ✨ Features

### 🌞 Light Monitoring

The **LDR sensor** measures the light intensity.

When the measured value drops below the configured threshold:

* The system considers the environment dark.
* The LEDs indicate the current farm state.
* The Green LED indicates that the servo-controlled mechanism is active.
* The Red LED indicates that the servo-controlled mechanism is inactive.

---

### ⚙️ Servo Motor Control

The servo motor can be controlled remotely through the MQTT `servo` topic.

For example:

```text
Topic: servo
Message: 180
```

The ESP32 moves the servo to **180°** and updates the system state.

The servo can also be returned to the closed position using the physical button.

---

### 🔊 Buzzer Control

The buzzer is controlled through the MQTT `buzzer` topic.

```text
Topic: buzzer
Message: 1
```

When the received value is `1`, the buzzer is activated.

The buzzer is also automatically turned off when the servo receives the `180°` command.

---

### 🚨 IR Sensor

The IR sensor is periodically read by the ESP32.

Every **1 second**, the current IR sensor reading is sent to Supabase.

Example data:

```json
{
  "id": 1,
  "value": 1234
}
```

---

### 🖥️ LCD Display

A **16×2 I2C LCD** provides simple system feedback.

The default display is:

```text
Threeshold farm
```

The display can also show temporary messages such as:

```text
Hello
```

and

```text
Thanks
```

---

### 🔘 Manual Button

The physical button provides manual control of the servo.

When the button is pressed while the servo is active:

1. The servo moves to `0°`.
2. The servo state changes to inactive.
3. The LCD displays a confirmation message.

---

## 🔌 Hardware Components

| Component    | Purpose                     |
| ------------ | --------------------------- |
| ESP32        | Main microcontroller        |
| LDR Sensor   | Light intensity detection   |
| IR Sensor    | Object / presence detection |
| Servo Motor  | Mechanical control          |
| Buzzer       | Audio alert                 |
| Red LED      | Status indication           |
| Green LED    | Status indication           |
| Push Button  | Manual servo control        |
| 16×2 I2C LCD | System status display       |

---

## 📍 ESP32 Pin Configuration

| Component   | ESP32 GPIO |
| ----------- | ---------: |
| Servo Motor |    GPIO 14 |
| LDR         |    GPIO 34 |
| Buzzer      |    GPIO 12 |
| IR Sensor   |    GPIO 32 |
| Red LED     |    GPIO 26 |
| Green LED   |    GPIO 27 |
| Push Button |    GPIO 25 |

---

## ☁️ Cloud & Communication

### MQTT — HiveMQ

The project uses **HiveMQ Cloud** for MQTT communication.

The ESP32 connects securely using:

```text
MQTT Port: 8883
Protocol: MQTT over TLS
```

The main MQTT topics are:

```text
servo
buzzer
```

### Servo Topic

```text
servo → 180
```

Moves the servo to 180 degrees.

### Buzzer Topic

```text
buzzer → 1
```

Activates the buzzer.

---

## 🗄️ Supabase Integration

The ESP32 uses **Supabase** to store IR sensor readings.

The `IR` table receives a new reading every second.

The stored data can later be used for:

* 📊 Data visualization
* 📈 Sensor monitoring
* 🔍 Historical analysis
* 🤖 Future machine-learning applications

---

## 🔄 System Workflow

```text
ESP32 Starts
     │
     ▼
Connect to Wi-Fi
     │
     ▼
Initialize Supabase
     │
     ▼
Connect to MQTT
     │
     ▼
Read Sensors
     │
     ├──── LDR ────► Check Light Threshold
     │
     ├──── IR ─────► Store Reading in Supabase
     │
     └──── Button ─► Control Servo
     │
     ▼
Receive MQTT Commands
     │
     ├──── servo ───► Move Servo
     │
     └──── buzzer ──► Activate Buzzer
     │
     ▼
Update LEDs & LCD
```

---

## 🛠️ Technologies Used

* **ESP32**
* **Arduino / C++**
* **MQTT**
* **HiveMQ Cloud**
* **Supabase**
* **Wi-Fi**
* **I2C LCD**
* **Servo Motor**
* **LDR Sensor**
* **IR Sensor**

### Libraries

```cpp
Arduino.h
LiquidCrystal_I2C.h
ESPSupabase.h
WiFi.h
PubSubClient.h
WiFiClientSecure.h
```

---

## 🚀 Getting Started

### 1. Clone the Repository

```bash
git clone <YOUR_REPOSITORY_URL>
cd <YOUR_REPOSITORY_FOLDER>
```

### 2. Configure Wi-Fi

Update the Wi-Fi credentials in the Arduino code:

```cpp
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";
```

### 3. Configure MQTT

Add your HiveMQ Cloud credentials:

```cpp
const char* mqtt_server = "YOUR_MQTT_SERVER";
const int mqtt_port = 8883;
const char* mqtt_user = "YOUR_MQTT_USERNAME";
const char* mqtt_pass = "YOUR_MQTT_PASSWORD";
```

### 4. Configure Supabase

Add your Supabase project URL and API key:

```cpp
String supabase_url = "YOUR_SUPABASE_URL";
String anon_key = "YOUR_SUPABASE_ANON_KEY";
```

### 5. Upload the Code

Open the project using **Arduino IDE** or **PlatformIO**, select the correct ESP32 board and COM port, then upload the firmware.

---

## ⚠️ Security

**Do not upload real credentials to GitHub.**

Avoid committing:

* Wi-Fi passwords
* MQTT usernames/passwords
* Supabase API keys
* Private certificates
* Other sensitive credentials

Instead, use placeholders:

```cpp
YOUR_WIFI_PASSWORD
YOUR_MQTT_PASSWORD
YOUR_SUPABASE_ANON_KEY
```

If credentials have already been pushed to a public repository, **rotate/change them immediately**.

---

## 🔮 Future Improvements

Possible improvements include:

* 🌡️ Add temperature and humidity sensors
* 💧 Add soil moisture monitoring
* 🚿 Automatic irrigation control
* 📱 Mobile application
* 📊 Real-time dashboard
* 🔔 Push notifications
* 🤖 Machine Learning for crop/environment prediction
* ⚡ Power consumption monitoring
* 🔐 Improved MQTT certificate validation
* 📈 Historical sensor analytics

---

## 🎯 Project Goal

The main goal of **Threeshold Farm** is to demonstrate how **IoT, cloud databases, MQTT communication, and embedded systems** can work together to build a practical smart agriculture solution.

The project provides a foundation that can be expanded into a larger **Smart Agriculture / Precision Farming** platform.

---

## 👨‍💻 Team

**Threeshold Farm Team**

* **Hamza Osama Mohamed** — Team Leader, Supabase Integration & Presentation
* **Mohaned Khaled** — Cloud Integration & Demo
* **Youssef Hussein** — ESP32 Code, Wiring & Wokwi

---

## 📄 License

This project was developed for educational and academic purposes.
