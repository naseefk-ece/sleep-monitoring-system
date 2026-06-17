# 🫁 Sleep Monitoring System

> An IoT-based wearable biomedical monitoring system that detects sleep apnea and nocturnal seizures in real time using multi-parameter physiological sensing — built as an affordable alternative to hospital-based polysomnography.

-----

## 📸 Project Demo

|Wearable Device             |Live Web Dashboard                |Session CSV Data      |
|:--------------------------:|:--------------------------------:|:--------------------:|
|![Device](images/device.jpg)|![Dashboard](images/dashboard.png)|![CSV](images/csv.png)|

-----

## 🧠 How It Works

### Sleep Apnea Detection

An apnea event is triggered when **all three** conditions are met simultaneously:

- SpO₂ drops **below 90%**
- Heart rate variation exceeds **15% from baseline**
- RR interval irregularity detected via AD8232

Each episode lasting **≥ 10 seconds** counts as one apnea event.

**AHI (Apnea-Hypopnea Index):**

```
AHI = Total Apnea Events / Hours of Sleep
```

|AHI Value|Severity|
|---------|--------|
|< 5      |Normal  |
|5 – 15   |Mild    |
|15 – 30  |Moderate|
|> 30     |Severe  |

### Nocturnal Seizure Detection

A composite **Seizure Metric (SM)** is computed from three signals:

```
SM = ECG_flag + SpO2_flag + sqrt(ax² + ay² + az²)
```

Where M (motion energy) = √(ax² + ay² + az²)

If SM exceeds the threshold → **seizure alert triggered**

-----

## 🔧 Hardware Components

|Component      |Model        |Function                                  |
|---------------|-------------|------------------------------------------|
|Microcontroller|ESP32        |Dual-core processing, Wi-Fi, sensor fusion|
|Optical Sensor |MAX30102     |Heart rate (PPG) & blood oxygen (SpO₂)    |
|ECG Front-End  |AD8232       |Single-lead ECG, heart rhythm, RR-interval|
|IMU            |MPU6050      |6-axis accelerometer for motion & tremor  |
|Display        |SSD1306 OLED |Local real-time vitals                    |
|Alarm          |Active Buzzer|Emergency alert when thresholds exceeded  |

-----

## 📡 System Architecture

```
[MAX30102] ──── PPG/SpO₂ ────┐
[AD8232]  ──── ECG/BPM  ────┤──► ESP32 ──► OLED Display
[MPU6050] ──── Motion   ────┘        │──► Buzzer Alarm
                                      │──► Wi-Fi Dashboard
                                      └──► ThingSpeak Cloud (optional)
```

**Core Assignment (FreeRTOS):**

- **Core 0** → Web server + ThingSpeak cloud upload
- **Core 1** → Live sensor pipeline (ECG, PPG, IMU, FFT)

-----

## 📊 Parameters Monitored

|Parameter            |Sensor  |Clinical Purpose                         |
|---------------------|--------|-----------------------------------------|
|Heart Rate (PPG)     |MAX30102|Cardiac activity via photoplethysmography|
|SpO₂                 |MAX30102|Blood oxygen saturation                  |
|ECG BPM & RR Interval|AD8232  |Heart rhythm analysis                    |
|Body Motion          |MPU6050 |Sleep movement patterns                  |
|Tremor / Jerk        |MPU6050 |Seizure activity via FFT                 |
|AHI                  |Computed|Apnea severity classification            |

-----

## 🌐 Wi-Fi Dashboard Features

- Live vitals: ECG/PPG heart rate, SpO₂, AHI, seizure metric
- Color-coded alarm cards (🟢 Normal → 🔴 Apnea → 🟣 Seizure)
- Session data stored in browser localStorage (up to 5,000 records)
- **Download CSV** for clinical review and long-term analysis
- **Reset Live Sensors** button to restart AHI counter and alarms
- Mobile-responsive layout

**To Access Dashboard:**

1. Connect your phone/laptop to Wi-Fi: `ESP32-Test` / Password: `12345678`
1. Open browser → `http://192.168.4.1`

-----

## 🛠️ Signal Processing Pipeline

### ECG (500Hz sampling)

1. **Moving Average Filter** (10-sample window) — removes 50Hz mains noise
1. **EMA Smoothing** (α = 0.3) — further noise reduction
1. **R-wave Peak Detection** — 300ms refractory period to count heartbeats

### Breathing / Apnea (10Hz FFT)

1. IMU chest tilt angle sampled at 10Hz
1. 64-point FFT with Hamming window
1. Peak magnitude extracted in **0.1–1.0 Hz** band (6–60 breaths/min)
1. No signal for **10 seconds** → apnea event logged

### Seizure Detection (10Hz)

- Jerk = change in acceleration between samples
- `SeizureIndex = (|ΔAx| + |ΔAy| + |ΔAz|) × 100`
- `SeizureIndex > 80` → seizure alert

-----

## 📁 Repository Structure

```
├── sleep_apnea_seizure_monitor.ino   # Main Arduino sketch (fully commented)
├── README.md                          # This file
├── images/
│   ├── device.jpg                     # Wearable prototype photo
│   ├── dashboard.png                  # Web dashboard screenshot
│   └── csv.png                        # Sample CSV data screenshot
└── docs/
    └── IoT-Based-Smart-Sleep-Monitoring-System.pptx  # Project presentation
```

-----

## ⚙️ Setup & Installation

### Prerequisites

Install these libraries in Arduino IDE (Sketch → Include Library → Manage Libraries):

```
Adafruit SSD1306
Adafruit GFX Library
SparkFun MAX3010x Pulse and Proximity Sensor Library
arduinoFFT
```

Board: **ESP32 Dev Module** (Tools → Board → ESP32 Arduino)

### Configuration

Open `sleep_apnea_seizure_monitor.ino` and update:

```cpp
// Line ~45 — Your router credentials (optional, for ThingSpeak cloud)
const char* router_ssid     = "YOUR_ROUTER_SSID";
const char* router_password = "YOUR_ROUTER_PASSWORD";

// Line ~50 — ThingSpeak API key (optional)
String apiKey = "YOUR_THINGSPEAK_WRITE_API_KEY";
```

### Pin Connections

|ESP32 Pin    |Connected To             |
|-------------|-------------------------|
|GPIO 21 (SDA)|OLED + MAX30102 + MPU6050|
|GPIO 22 (SCL)|OLED + MAX30102 + MPU6050|
|GPIO 34      |AD8232 OUTPUT            |
|GPIO 4       |AD8232 LO-               |
|GPIO 5       |AD8232 LO+               |
|GPIO 26      |Active Buzzer (+)        |

### Flash & Run

1. Connect ESP32 via USB
1. Select correct COM port
1. Upload sketch
1. Open Serial Monitor (115200 baud) to see IP address
1. Connect to `ESP32-Test` hotspot → open `http://192.168.4.1`

-----

## 👥 Team

|Name       
|-----------
|Naseef K
|Athul P     
|Sreegouri  
|Nasiha Nowshad


**Date:May 29 - June 12 2026

-----

## 📄 License

This project is open-source for academic and research purposes.

-----

*Built as a low-cost, portable alternative to hospital polysomnography — making sleep disorder diagnosis accessible anywhere.*
