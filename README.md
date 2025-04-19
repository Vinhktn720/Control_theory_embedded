# Control Theory on STM32F4

This project demonstrates the application of control theory algorithms—such as PID, fuzzy logic, and others—on embedded systems using the STM32F4 microcontroller.

The goal is to bring real-time control strategies, often used in robotics and automation, into an embedded C environment. This is ideal for students, researchers, or hobbyists who want to experiment with implementing control theory directly on hardware.

---

## 📦 Project Structure
Control_theory_embedded/ ├── Core/ # Main C source and header files │ ├── Inc/ # Header files │ └── Src/ # Source files (main logic, controllers) ├── Drivers/ # STM32 HAL and custom drivers ├── Fuzzy_PID/ # Fuzzy PID controller implementation ├── PID/ # Classic PID controller ├── ... # Other control logic modules ├── .ioc # STM32CubeMX project file ├── Makefile / .cproject # For building with Make or IDEs └── README.md
## ⚙️ Features

- ✅ Classic PID controller (tunable Kp, Ki, Kd)
- ✅ Fuzzy logic-based PID controller
- ✅ Real-time control loop execution on STM32F4
- ✅ Modular and reusable codebase
- ✅ Easy debugging via UART / serial interface

---

## 🛠️ Getting Started

### Requirements

- STM32F4 Discovery board (e.g., STM32F407VG)
- STM32CubeIDE or STM32CubeMX (for configuring peripherals)
- USB to UART cable (for debugging/communication)
- `st-link` tools (optional for Linux users)
- Basic understanding of embedded C and control theory

### Flashing the Project

1. Clone the repo:
   ```bash
   git clone https://github.com/Vinhktn720/Control_theory_embedded.git
   cd Control_theory_embedded

       Open the .ioc file with STM32CubeMX or directly import the project into STM32CubeIDE.

    Generate code and build the project.

    Flash the binary to your STM32F4 board.

📈 Example Use Cases

    Control motor speed or position

    Balance a pendulum

    Temperature control with sensor feedback

    Implement fuzzy logic-based adaptive control

🧠 Control Theory Algorithms
Algorithm	Description
PID	Classic control loop with tuning parameters
Fuzzy PID	Adaptive PID using fuzzy logic rules
Feedforward	(Optional future module)
📡 Communication

Data can be sent via UART to visualize system behavior or tune the controller in real-time (e.g., using a Python script or serial plotter).
🚧 Future Plans

Add more control strategies (e.g., LQR, MPC)

Real-time parameter tuning via serial

Python GUI to visualize system response

    Simulink model for simulation-to-code workflow

🙌 Credits

Developed by Vinhktn720
Inspired by classic control systems and embedded robotics.
📄 License

MIT License. See LICENSE for more info.
