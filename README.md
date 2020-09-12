![SinESC Logo](https://raw.githubusercontent.com/SAR-mango/SinESC/master/Logo/Logo.png)
# Overview
SinESC is an electronic speed controller (ESC) that supports full sinusoidal control of tri-phase brushless DC motors. Targeted towards RC drone and wing pilots.

There are two versions of SinESC. Wing Edition and Multi Edition:
- Wing Edition supports CAN and standard PWM control signals. CAN is intended for Pixhawk users, and standard PWM is for direct connection to an RC receiver.
- Multi Edition communicates with a flight controller though *SinWire*, a custom 1-wire protocol that enables configuration, firmware updates, and speed control all through one wire. SinWire support will be added to Betaflight soon. It aims to match or exceed the update rate of DSHOT600.

For specs, look further below.
![Multi Edition v2.3B Top](https://raw.githubusercontent.com/SAR-mango/SinESC/master/Multi%20Edition/2.3B/SinESC-Multi-2.3B/Exported%20Files/3D%20Render%20Front.jpg)
![Multi Edition v2.3B Bottom](https://raw.githubusercontent.com/SAR-mango/SinESC/master/Multi%20Edition/2.3B/SinESC-Multi-2.3B/Exported%20Files/3D%20Render%20Back.jpg)
# Benefits of Sinusoidal Control
Brushless DC motors work best when their phase voltage waveforms are sinusoid-shaped. Try it yourself—connect a BLDC motor to an oscilloscope and spin the rotor with your fingers. You will see a sine wave. 

Regular BLHELI_32 ESCs use a simple "six-point" motor control method in which the phases are energized to bring the rotor to one of six points on a circle. This method is easy to implement as the rotor position at each point will result in a zero-crossing in the back-EMF from the non-energized phase(s), which is easy to detect. This means that the phase voltage waveforms are trapezoid-shaped rather than sinusoid-shaped. This is not ideal as the fields are not *always* aligned with the rotor to maximize torque. This causes the rotor to effectively "jerk in a circle" rather than spin continuously, resulting in lower efficiency and rougher flight. Furthermore, the sharp edges of the trapezoidal signal also introduce high-frequency noise which contributes to the above disadvantages as well. This also makes the motors run louder and hotter.

Sinusoidal control minimizes torque ripple, maximizing efficiency. The magnetic fields are aligned precisely to exert the maximum torque on the rotor at any given position and power. Minimized torque ripple leads to drastically **smoother flight**. Greater efficiency (*SinESC achieves **> 97% efficiency**; trapezoidal ESCs typically achieve 70%*) allows for **longer flight times** and lower peak currents, which can increase battery longevity as well.
# Features & Specifications
- Bus voltage monitoring for power measurement and under-voltage protection
- 60kHz PWM frequency—with sinusoidal control, PWM frequency hardly affects torque and smoothness. 30kHz and above feel the same. Consequently, 60kHz was selected as it offered maximum efficiency.
- Broken-out and labeled debug pins for hackers
- **Maximum 6S Li-po battery (25.2V)**
- **Maximum 40A continuous, 50A burst current draw (5 seconds).**
  - With the greatly-increased efficiency of SinESC, peak motor currents will be *significantly lower* than before at a given throttle/thrust level. *THIS DOES NOT MEAN YOUR MOTORS WILL RUN WEAKER. THEY ARE JUST MORE EFFICIENT!*
  - *Over-current protection takes advantage of comparators embedded in the microcontroller and triggers immediately at 55A.*
# Current-Sense Topology
Not just any ESC can perform sinusoidal control, and not all sinusoidal ESCs are created equal. Sinusoidal control requires that the ESC know the exact position of the rotor in order to properly align the magnetic field. Perfect alignment results in a perfect sine wave.

There are a few ways to find the position of the rotor:
- The first, most obvious method is to put a hall-effect (or similar) sensor on the motor to determine exactly where the rotor is. This sensor simply feeds data to the ESC. However, this is impractical for drones and wings due to durability, size, and weight concerns.
- The second way is to calculate the position of the rotor based on the currents flowing through each phase. Technically, measuring just the total motor current is sufficient, but this leads to poor position estimation and thus reduced performance. Alternatively, each half-bridge can connect to ground through a series current shunt and the voltage drop across each shunt can be amplified and read by an ADC in a microcontroller. This allows for the current through each phase to be known, resulting in far superior performance. Though more complicated, this is the current-sensing topology that SinESC employs.
- Finally, inline current-sensing is also possible. This technique involves placing shunts in series with only two of the three motor phases, since the third phase current can be calculated with Kirchoff's Current Law. However, this requires complicated, expensive amplifiers as high common-mode rejection ratio is critical. Some gate driver ICs integrate these amplifiers, but their gain cannot be tuned to take full advantage of the ADC resolution. At high currents, using the full ADC resolution is critical. Thus, the second method was selected.
# Hardware Achievements
The four-layer PCB layout of SinESC is the main accomplishment of this project:

The 15x30mm SinESC Multi Edition PCB uses 0201-sized components when possible for reduced parasitic influences and reduced size. Maximum component density is approximately *50 components/cm²*. The microcontroller, gate driver, and current-sense amplifiers (all the control electronics) are implemented on the top layer, leaving the bottom layer for the power electronics. This provides room for wide traces. At an increased size of 17.5x35mm, SinESC Wing Edition adds CAN support with a robust transceiver.

Multi Edition is designed in KiCAD and adheres to OSHPark design rules. All additional libraries are within the project files. Wing Edition was designed in EasyEDA and will soon be ported to KiCAD (a major redesign is required). It currently adheres to the inferior JLCPCB design rules.

*For eventual production, both versions may be redesigned to 6 mil trace/space design rules. 4 oz copper thickness and blind/buried vias would be supported.*

SinESC is designed specifically for efficiency and robustness:
- ST Microelectronics STM32F303CBT7 microcontroller
- Maxim Integrated MAX4239 precision op-amps for current-sensing. Incredibly low input offset voltage (*0.1µV*). Gain = 30V/V; offset tuned to use the full ADC resolution.
- Low-resistance current shunts (*1mΩ*) for reduced power loss
- Switching regulator provides logic-level voltage supply for control electronics (*efficiency > 80%*)
- Thermally optimized PCB layout
- Trinamic Motion Control TMC6100-LA gate driver
  - Fast switching slopes achieved with high gate drive current (1.5A), decreasing switching losses
- Infineon Technologies BSC010N04LSI N-channel MOSFETs
  - Extremely low on-resistance (*Rdson = 1.05mΩ*) for reduced power loss
  - High drain-source voltage (*Vds = 40V*) for immunity to voltage spikes even at 6S Li-po voltages
- High-quality passive components
  - TDK ceramic capacitors
  - Panasonic metal film resistors (all 1% tolerance including shunts)
# Versions
Regular Semantic Versioning is used for all software components of the project. As for PCB designs, Semantic Versioning has been modified. Versions only reflect the PCB design since schematic changes directly influence the PCB. They are written like so:

vX.YZ where X and Y are numbers and Z is a letter. "v" is sometimes omitted.
- If a minor change is made that does *NOT* require a new stencil *OR* an updated BOM, Z is incremented.
- If a minor change requires a new stencil *and/or* an updated BOM, Y is incremented and Z is reset. Ideally, few minor changes are made between major releases.
- If a major change is made, X is incremented and Y and Z are reset.
