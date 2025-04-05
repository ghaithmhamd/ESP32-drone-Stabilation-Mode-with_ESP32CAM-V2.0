# ESP32-drone-Stabilation-Mode-with_ESP32CAM-V2.0

🚁 A DIY drone stabilization system using ESP32-S3-Zero and ESP32-CAM, designed for lightweight, real-time flight control and video feedback. Combines sensor fusion, PID control, and NRF communication for stable and responsive quadcopter performance.

---

## 🛠️ Hardware Used

- **ESP32-S3-Zero** – Main flight controller  
- **ESP32-CAM** – WiFi video streaming module  
- **MPU6050** – IMU (accelerometer + gyroscope)  
- **BMP280** – Barometric pressure sensor (altitude)  
- **NRF24L01** – Wireless communication  
- **4× ESCs + Brushless Motors** – Power system  
- **3S LiPo Battery** – Power supply  
- **Custom Transmitter** – [Radio Transmitter Repository](https://github.com/ghaithmhamd/Radio-transmitter-and-reciever)

---

## 🎯 Features

- ✔️ PID-based drone stabilization  
- ✔️ Kalman filter for **MPU6050 accel + gyro fusion**  
- ❌ Vertical velocity fusion (accel Z + BMP280) **not yet implemented**  
- 📡 NRF24L01-based remote control  
- 📷 Real-time video via ESP32-CAM (WiFi)  
- 🔋 Powered by compact 3S LiPo battery  
- 🌲 Designed for green environment surveillance (e.g., forest monitoring)

---

## 📸 Demo & Media

- 🔗 **ESP32-CAM preview**: [YouTube video](https://youtu.be/JYchUapoqzc?si=Sv1O5FwJmP0YOA6_)  
- 🎥 **Full flight test video**: [Facebook video](https://www.facebook.com/share/p/1HFidd3Syk/)  
- 🎮 **Transmitter**: [GitHub - Radio Transmitter & Receiver](https://github.com/ghaithmhamd/Radio-transmitter-and-reciever)

---

## 🧠 To-Do / Improvements

- [ ] Implement vertical velocity Kalman fusion (MPU6050 Z-axis + BMP280)  
- [ ] Add GPS and waypoint navigation  
- [ ] Telemetry feedback (e.g., via Bluetooth or WiFi)  
- [ ] Mobile app integration for FPV + controls  
- [ ] Add support for auto-takeoff and landing

---

## Installation & Setup
1. Clone the repository.
2. Upload the code to your **ESP32-S3-Zero** using **Arduino IDE**.
3. Ensure proper sensor calibration before flight.
4. Connect the transmitter (linked above) and test the response.

## Notes
- Ensure that the **IMU is properly mounted** to avoid drift.
- The **PID parameters** might need tuning based on your drone's configuration.

## References
- [pratikphadte](https://www.youtube.com/@pratikphadte) 
- [Carbon Aeronautics](https://youtube.com/@carbonaeronautics?si=-DZ1Sz5sgNruoJgR)

## License

HSRmh(HardSoftRoboticsMh) License

Copyright (c) 2024 pratikPhadte

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
