# Rocket-Fin-Stabilizer
Arduino-based rocket fin stabilization system using MPU6050 and servo actuators

# Features

- Real-time Attitude Control: 20Hz update rate for responsive stabilization

- 4-Fin Configuration: Independent control of pitch and roll axes

- Auto Calibration: Automatic sensor offset calculation on startup

- Deadband Control: Prevents servo jitter during stable flight

- Serial Monitoring: Real-time telemetry output for debugging

- Arduino Mega Optimized: Utilizes additional pins and processing power

# Hardware Requirements
## Components Required

| Component         | Specification            | Quantity |
|------------------|--------------------------|----------|
| Arduino Mega 2560| ATmega2560, 16MHz         | 1        |
| MPU6050          | 6-axis IMU sensor         | 1        |
| Servo Motors     | SG90 or similar, 9g       | 4        |
| Power Supply     | 11.1V LiPo recommended     | 1        |


## Pin Configuration (Arduino Mega)

### MPU6050:
- SDA → Pin 20

- SCL → Pin 21  

- VCC → 5V

- GND → GND

### Servos:
- Fin 1 → Pin 8

- Fin 2 → Pin 9

- Fin 3 → Pin 10

- Fin 4 → Pin 11

# Circuit Diagram
![image](https://github.com/user-attachments/assets/b031b0c0-2b4a-45d8-8015-d41e9feb3a42)


# Theory of Operation
The stabilization system uses a closed-loop control approach:

- Sensor Reading: MPU6050 provides accelerometer data at 20Hz
- Attitude Calculation: Roll and pitch angles computed using accelerometer
- Error Calculation: Deviation from level flight determined
- Control Output: Servo positions calculated to counteract rotation
- Actuation: Fins moved to generate corrective aerodynamic forces

## Control Logic
Servo Position = Center ± (Angle × Gain)
# Installation

```bash
git clone https://github.com/diptiman-mohanta/rocket-fin-stabilizer.git
cd rocket-fin-stabilizer
```
# Citation

If you use this work in your research, please cite:

```bibtex
@misc{rocket-fin-stabilizer,
  title={Rocket fin stabilizer using Arduino, MPU6050 and Servos},
  author={Diptiman Mohanta and Akash S R},
  year={2025},
  url={https://github.com/diptiman-mohanta/rocket-fin-stabilizer.git}
}
```
