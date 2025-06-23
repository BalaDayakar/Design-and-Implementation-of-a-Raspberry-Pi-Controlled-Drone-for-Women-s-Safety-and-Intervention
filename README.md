# 🚁 Women Safety Drone using Raspberry Pi & SIM800L

A real-time **autonomous drone system** developed to respond to emergency SOS alerts for enhancing women's safety. This project integrates **Raspberry Pi**, **GPS**, **SIM800L GSM module**, **Camera**, and a **Flight Controller** to monitor, locate, and assist users in distress through navigation, surveillance, and audible alerts.

---

## 🧠 Project Summary

This drone is equipped with:
- Autonomous flight capability
- GPS tracking from SMS
- Real-time video surveillance
- Siren-based alert mechanism
- Return-to-launch functionality

All powered by **DroneKit**, **PyMavlink**, and Raspberry Pi GPIO controls.

---

## 📦 Features

- 📍 **Reads GPS Coordinates** from SMS (Google Maps link)
- 🛫 **Autonomous Takeoff & Navigation** to victim’s location
- 🎥 **Live Camera Support** for surveillance (optional)
- 🔊 **Relay-Triggered Siren** to alert people nearby
- 📡 **SIM800L Integration** for real-time SMS communication
- 🔁 **Auto Return-To-Launch (RTL)** after mission
- 💡 **Expandable for AI-Based Threat Detection**

---

## 🛠️ Technologies Used

- `Python 3`
- `DroneKit`, `PyMavlink`
- `RPi.GPIO`
- `Geopy` (distance calculation)
- `Serial` (GSM communication)
- `MAVProxy`, `Mission Planner`
- `Raspberry Pi 3B+/4`, `SIM800L`, `Pixhawk/Cube Orange`

---

## 🧾 Requirements

- Raspberry Pi 3B+/4 (with Raspbian OS)
- Cube Orange or Pixhawk Flight Controller
- SIM800L GSM Module
- BLDC motors + ESCs
- GPS Module (Here 3/3+ recommended)
- Camera (USB/PiCam, optional)
- Relay Module + Siren
- 4G/Wi-Fi dongle (optional for video transmission)
- DroneKit, MAVProxy installed on Pi

---

## ⚙️ Setup Instructions

### 1. 🔌 Hardware Connections
- SIM800L → `/dev/ttyAMA0`
- Relay to GPIO `17` on Pi
- Pixhawk connected via USB to Raspberry Pi
- Siren connected through Relay module

### 2. 💾 Software Installation on Raspberry Pi

```bash
sudo apt-get update && sudo apt-get upgrade
sudo apt-get install python3-pip python3-dev
sudo pip3 install future pyserial dronekit MAVProxy geopy
