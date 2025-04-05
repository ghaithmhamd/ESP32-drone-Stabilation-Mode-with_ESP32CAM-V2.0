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

## 📩 Contact

For updates, questions, or to see live demos, visit the [Facebook page](https://www.facebook.com/profile.php?id=61574058525266&locale=fr_FR).
