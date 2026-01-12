# Multi-Sensor-IoT-Security-System-for-Art-Conservation
Multi-Sensor IoT Security System for Art ConservationA real-time, multi-threaded security monitoring system designed for high-value asset protection (e.g., museum art pieces). This system utilizes a Raspberry Pi Zero 2W integrated with a suite of I2C sensors to detect environmental anomalies and unauthorized movement.

📝 Technical Motivation
This project explores the integration of deterministic sensor data with asynchronous web services. By implementing custom I2C drivers and multi-threaded monitoring loops, the system achieves industrial-grade responsiveness in detecting physical state changes—a critical requirement for high-stakes conservation environments.


🌟 Key Features
# Multi-Dimensional Sensing: 
Monitors light intensity (VEML7700), atmospheric pressure (SPL06), distance (VL5300 ToF), and temperature/humidity (SHT4x).
# Custom Sensor Drivers: 
Includes a high-precision implementation for the SPL06-001 pressure sensor to detect subtle air pressure changes.
# Intelligent Alarm Logic:
    Baseline Calibration: Establishes environmental "normal" states upon arming the system.
    Deviation Detection: Triggers alerts when real-time data drifts beyond predefined physical thresholds.
# Automated Evidence Logging: 
Upon alarm, the system initiates a 120-second continuous image capture sequence using Picamera2 for forensic evidence.
# Asset Identification:
Integrated QR code recognition (ZBar) to verify and log specific art pieces during the scanning phase.
# Live Dashboard:
A Flask-based web interface providing real-time telemetry, live video feed, and historical data visualization.

🛠️ Hardware Stack
# Controller: 
Raspberry Pi Zero 2W
# Sensors:
Distance: VL5300 ToF (Time-of-Flight) Laser Ranging
Pressure: SPL06-001 (hPa)
Environment: SHT40 (Temp/Humid)
VEML7700 (Lux)
Safety: Smoke Sensor & Active Buzzer
Camera: Raspberry Pi Camera Module (Picamera2)

💻 Software Architecture
Language: Python 3.x
Framework: Flask (Web UI)
Concurrency: Multi-threading for simultaneous sensor polling, camera streaming, and web hosting.
Protocols: I2C (via smbus2 and busio).

🚀 Installation & Usage

1.Clone the Repository:
'git clone https://github.com/your-username/IoT-Art-Security.git'

2.Install Dependencies:
'pip install flask smbus2 adafruit-circuitpython-sht4x adafruit-circuitpython-veml7700 pyzbar picamera2'

3.Run the Application:
'python security.py'

4.Web Access: 
Open http://<your-pi-ip>:5000 to access the security dashboard.

