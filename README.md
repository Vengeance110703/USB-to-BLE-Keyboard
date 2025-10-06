# 🔌 USB-to-BLE Keyboard (ESP32-S3)

This project transforms an **ESP32-S3** into a bridge between a **USB keyboard** and **Bluetooth HID**, allowing any wired USB keyboard to function as a wireless BLE keyboard.  
Simply plug your keyboard into the ESP32-S3 and connect your phone, tablet, or PC via Bluetooth.

---

## 🚀 Features

- **USB Host Mode** – Detects and reads keypresses from standard USB HID keyboards via TinyUSB.
- **BLE HID Emulation** – Sends keystrokes over Bluetooth as a standard HID keyboard.
- **Status LED Indicators:**

  | LED Color    | Meaning                     |
  | ------------ | --------------------------- |
  | 🔴 **Red**   | USB & BLE both disconnected |
  | 🟢 **Green** | USB keyboard connected      |
  | 🔵 **Blue**  | BLE device connected        |
  | ⚪ **White** | Both USB and BLE connected  |

- Supports **modifier keys** (Ctrl, Alt, Shift) and up to six simultaneous keypresses.
- Sends "key release" events to ensure **proper key repetition** behavior.
- Built entirely using ESP-IDF, TinyUSB (USB Host), and BLE HID stack.

---

## 🧩 Project Structure

```md
USB-to-BLE-Keyboard/
├── main
│ ├── CMakeLists.txt
│ ├── esp_hidd_prf_api.c
│ ├── esp_hidd_prf_api.h
│ ├── hid_dev.c
│ ├── hid_dev.h
│ ├── hid_device_le_prf.c
│ ├── hid_host_example.c
│ ├── hidd_le_prf_int.h
│ └── idf_component.yml
├── CMakeLists.txt
├── hid_usage_table_keyboard.pdf
├── sdkconfig
└── sdkconfig.defaults
```

---

### 📄 File Descriptions

#### **main/**

- **CMakeLists.txt** – Build configuration for the main component.
- **esp_hidd_prf_api.c** – Implements the BLE HID profile and handles HID report operations.
- **esp_hidd_prf_api.h** – Header file defining BLE HID profile APIs.
- **hid_dev.c** – Helper functions for HID report parsing and construction.
- **hid_dev.h** – Header definitions and constants for HID utilities.
- **hid_device_le_prf.c** – BLE HID device logic handling GATT communication and HID services.
- **hid_host_example.c** – Main application logic; initializes USB host and bridges data to BLE HID.
- **hidd_le_prf_int.h** – Internal constants, macros, and data structures for BLE HID profile.
- **idf_component.yml** – Metadata and dependency declarations for ESP-IDF components.

#### **Root Directory**

- **CMakeLists.txt** – Root-level ESP-IDF build configuration file.
- **hid_usage_table_keyboard.pdf** – Reference document listing HID key usage codes.
- **sdkconfig** – Build configuration file generated during compilation.
- **sdkconfig.defaults** – Default configuration template for project initialization.

---

## ⚙️ How It Works

1. **USB Initialization**  
   The ESP32-S3 acts as a USB host using TinyUSB to enumerate and communicate with the connected keyboard.

2. **HID Report Parsing**  
   Incoming USB HID reports are parsed to extract keycodes and modifier states.

3. **BLE HID Emulation**  
   The ESP32-S3 translates parsed data into BLE HID reports and sends them via GATT notifications to the connected Bluetooth device.

---

## 🧠 Usage Guide

### Requirements

- ESP32-S3 board with native USB OTG support
- ESP-IDF v5.x or later
- USB keyboard + OTG adapter/cable
- Serial monitor (for debugging)

### Build & Flash

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

---

## 🔗 Pairing

1. Power the **ESP32-S3** and connect a **USB keyboard**.
2. On your **PC or phone**, scan for Bluetooth devices.
3. Connect to the advertised **BLE keyboard**.

---

## ⚠️ Limitations

- Only supports **boot protocol keyboards** (standard USB HID).
- **BLE battery service** is not implemented as provision for a battery was not added.
- Some **multimedia or vendor-specific keys** may not work.

---

## 🔮 Future Improvements

- Add BLE battery service
- Support multimedia keys
- Extend to composite HID (keyboard + mouse)

---

## 🏗️ Built With

- **ESP-IDF** – Espressif IoT Development Framework
- **TinyUSB** – USB Host stack
- **ESP BLE HID Profile** – For Bluetooth keyboard emulation
