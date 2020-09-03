# Overview
SinESC brings sinusoidal motor control to the RC drone & wing communities while retaining standard features and low cost.

There are two versions of SinESC. Wing Edition and Multi Edition:

Wing Edition supports CAN and standard PWM control signals. CAN is intended for Pixhawk users, and standard PWM is for direct connection to an RC receiver.

Multi Edition communicates with a flight controller though SinWire, a custom 1-wire protocol that enables configuration, firmware updates, and speed control all through one wire.
SinWire support will be added to Betaflight soon. It aims to match or exceed the update rate of DSHOT600.
# Benefits of Sinusoidal Control: Efficiency & More!
Brushless DC motors work best when their phase voltage waveforms are sinusoid-shaped. Try it yourself—connect a BLDC motor to an oscilloscope and spin the rotor with your fingers. You will see a sine wave. 

Regular BLHELI_32 ESCs employ a simple motor control method known as "trapezoidal drive," in which the phase voltage waveforms are trapezoid-shaped rather than sinusoid-shaped. This is non-ideal as the fields are not always perfectly aligned with the rotor, which means that torque is not constantly maximized at a given power. This causes the rotor to effectively "jerk in a circle" rather than spin continuously, resulting in lower efficiency and rougher flight. Furthermore, the sharp edges of the trapezoidal signal also introduce high-frequency noise which contributes to the above disadvantages as well. This also makes the motors run louder and hotter.

Sinusoidal control minimizes torque ripple, maximizing efficiency. The magnetic fields are aligned precisely to exert the maximum torque force on the rotor at any given position. Minimized torque ripple leads to drastically smoother flight. Greater efficiency (SinESC achieves > 97% efficiency; trapezoidal ESCs typically achieve 70%) allows for longer flight times and lower peak currents, which can increase battery longevity as well.



SinESC is designed specifically for efficiency:

- Low-resistance current shunts (1mΩ) for minimal power loss
- Switching regulator provides logic-level voltage supply for all control electronics
- Extremely low on-resistance MOSFETs selected for the power stage (Rdson = 1.05mΩ)
- Fast switching slopes achieved with high gate-drive current, minimizing switching losses
- Thermally optimized PCB layout

# Hardware Achievements
Not just any ESC can perform sinusoidal control, and not all sinusoidal ESCs are created equal. Sinusoidal control requires that the ESC know the exact position of the rotor in order to properly align the magnetic field. Perfect alignment means a perfect sine wave.

There are a few ways to find the position of the rotor:

The first, most obvious method is to put a (hall effect or similar) sensor on the motor to determine exactly where the rotor is. This sensor simply feeds data to the ESC. However, this is impractical for drones and wings due to durability, size, and weight concerns.

The second way is to calculate the position of the rotor based on the currents flowing through each phase. Technically, measuring just the total motor current is sufficient, but this leads to poor rotor position estimation and thus reduced performance. Alternatively, each half-bridge can connect to ground through a series current shunt and the voltage drop across each shunt can be amplified and read by an ADC on a microcontroller. This allows for the current through each phase to be known, resulting in superior performance. Though more complicated, this is the current-sensing topology that SinESC employs.

The PCB Layout of SinESC Multi Edition is the main accomplishment of this project:

The 15x30mm PCB size of SinESC Multi Edition uses 0201-sized components when possible for reduced parasitic influences and, of course, reduced size. Component density is approximately 86 components/square inch. The microcontroller, gate driver, and current-sense amplifiers (all the control electronics) are implemented on the top layer, leaving the bottom layer for the power electronics. This provides room for wide traces.

At an increased size of 17.5x35mm, SinESC Wing Edition adds CAN support with a robust transceiver.

# Standard Features
Both versions of SinESC include the following standard features:

- Hardware over-current protection (takes advantage of the comparators embedded in the STM32F3 microcontroller)
- Bus voltage monitoring for power monitoring and under-voltage protection
- Typical configuration options such as motor direction
- 60kHz PWM frequency for smooth flight. Increased PWM frequencies show no benefit and only decrease efficiency.
- Easy installation and configuration. No different than any standard BLHELI_32 ESC.
- Broken-out and labeled debug pins for hackers
- For those who wish to repair a broken ESC, information and support will always be provided.

# Key Components
- ST Microelectronics STM32F303CBT7 microcontroller
- Trinamic Motion Control TMC6100-LA gate driver with SPI; gate drive current set to maximum of 1.5A
- Maxim Integrated MAX4239 precision op-amps for current-sensing. Incredibly low input offset voltage (0.1µV). Gain = 30V/V; offset tuned to use the full ADC resolution.
- Infineon Technologies BSC010N04LSI N-channel MOSFETs
- High-quality passives
  - TDK ceramic capacitors
  - Panasonic metal film resistors; all 1% tolerance including shunts
