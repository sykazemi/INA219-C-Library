
```markdown
# INA219 STM32 HAL Driver

A robust, modular and configurable driver for the **INA219** current, voltage, and power sensor using **STM32 HAL**, written in industrial-grade style.

---

## 📌 Features

- Full support for configuration of:
  - Operating Mode
  - Bus Voltage Range
  - Gain
  - ADC Resolution & Averaging (Shunt & Bus)
- Individual API functions for updating specific config fields without overwriting others
- Supports both **I2C hardware abstraction** and **FreeRTOS compatibility**
- Clean separation of Public API and Internal Logic
- Error handling stub included for future expansion
- Full documentation in code with structured `typedef`s

---

## 📁 File Structure

```

├── inc/
│   └── ina219.h              # Public API header
├── src/
│   └── ina219.c              # Driver implementation
├── example/
│   └── main.c                # Usage example (Coming soon)
└── README.md                 # This file

````

---

## ⚙️ API Usage

### 1. Initialization

```c
INA219_HandleTypeDef ina219 = { .hi2c = &hi2c2, .i2c_address = 0x40,
.shunt_resistance = 0.03f };

INA219_Init(&ina219);
````

---

### 2. Change Parameters Dynamically

```c
INA219_SetCalibration(&ina219, 1.0f);
INA219_SetBusADCResolution(&ina219, INA219_ADC_RES_12BIT_128S);
INA219_SetBusVoltageRange(&ina219, INA219_BUS_RANGE_32V);
INA219_SetGain(&ina219, INA219_GAIN_1_40MV);
INA219_SetMode(&ina219, INA219_MODE_SHUNT_BUS_CONTINUOUS);
INA219_SetShuntADCResolution(&ina219, INA219_ADC_RES_12BIT_128S);
```

---

### 3. Read Measurements

```c
float voltage = 0.0f, current = 0.0f;
INA219_ReadCurrent(&ina219, &current);
INA219_ReadVoltage(&ina219, &voltage);
```

---

## 🔧 Configuration Options

| Setting        | Options                                   |
| -------------- | ----------------------------------------- |
| Bus Voltage    | 16V, 32V                                  |
| Gain           | ±40mV to ±320mV (Gain x1 to x8)           |
| ADC Resolution | 9-bit to 12-bit, or averaging (up to 16x) |
| Mode           | Power-down, shunt/bus single/continuous   |

---

## 🚀 To Do

* [ ] Add example project (CubeMX + main.c)
* [ ] Add error handler customization
* [ ] Add optional calibration API

---

## 📜 License

MIT License. See `LICENSE` file.

---

## 👨‍💻 Author

**Seyed Yasin Kazemi**
Industrial Embedded Systems Developer
Contact: *\[Add contact info or GitHub profile link]*

```


