# Current State of the Project (January 2022)
I haven't been able to build any prototypes for the past year due to the ongoing chip shortage, but here is a list of changes I had planned to make:

- Switch to HDI PCB
- Switch to discrete half-bridge gate drivers
- Switch to a variant of the STM32G4 MCU
- Switch to modern 3.3x3.3mm MOSFETs with low gate charge, same 40V Vds, similar Rdson, and high avalanche energy rating
- Switch to integrated op-amps for current sensing

These changes mostly improve simplicity and robustness, and will make it easier to develop a 4-in-1 version of SinESC after the single version is fully complete. I haven't made these changes yet because I am waiting to see what parts are available once the appropriate STM32 microcontrollers are in stock. As of now, SinESC successfully spins motors with > 90% efficiency, but there are reliability concerns with the TMC6100 gate driver. Also, some software work is required to add DSHOT support. Finally, a more long-term goal is to develop a self-tuning application integrated with Betaflight that can be run to optimize SinESC for a specific motor.

# Overview
SinESC (pronounced "sign ESC," namesake being the sine wave) is an electronic speed controller (ESC) that supports full sinusoidal control of three-phase brushless DC motors used by RC drone pilots.

The real-life dimensions of the PCBs pictured below are 15x30mm.
![Multi Edition v2.3C Top](https://raw.githubusercontent.com/SAR-mango/SinESC/master/Multi%20Edition/SinESC-Multi/Exported%20Files/3D%20Render%20Front.jpg)
![Multi Edition v2.3C Bottom](https://raw.githubusercontent.com/SAR-mango/SinESC/master/Multi%20Edition/SinESC-Multi/Exported%20Files/3D%20Render%20Back.jpg)

# Features & Specifications*
- 90% efficiency (up to 40% longer flight)
- Smooth flight
- Quiet operation
- Bus voltage monitoring for power measurement and under-voltage protection
- Broken-out and labeled debug pins for hackers
- 3–6S Li-po battery support
- Maximum 30A continuous, 40A burst current draw
  - With the greatly-increased efficiency of SinESC, peak motor currents will be significantly lower than before (at a given throttle/thrust level). This does NOT mean your motors produce less power—they are just more efficient.
  - Over-current protection takes advantage of comparators embedded in the microcontroller and triggers immediately at 55A.

\*Several of the above specifications are currently only theoretical; they have not been confirmed with measurements. This page will be updated immediately when any specification changes or is confirmed.

# Sinusoidal Control: Benefits & a Brief Description
Brushless DC motors work best when their phase voltage waveforms are sinusoid-shaped. Try it yourself—connect a BLDC motor to an oscilloscope and spin the rotor with your fingers. You will see a sine wave. 

Regular BLHELI_32 ESCs use a simple "six-point" motor control method in which the phases are energized to bring the rotor to one of six points on a circle. This method is easy to implement as the rotor position at each point will result in a zero-crossing in the back-EMF from the non-energized phase(s), which is easy to detect. This means that the phase voltage waveforms are trapezoid-shaped rather than sinusoid-shaped. This is not ideal as the fields are not *always* aligned with the rotor to maximize torque. This causes the rotor to effectively "jolt in a circle" rather than spin continuously, resulting in lower efficiency and rougher flight. Furthermore, the sharp edges of the trapezoidal signal also introduce high-frequency noise which contributes to the above disadvantages as well. This also makes the motors run louder and hotter.

Sinusoidal control minimizes torque ripple, maximizing efficiency. The magnetic fields are precisely aligned to exert maximum torque on the rotor at *any* given position and power. Minimized torque ripple leads to smoother flight. Greater efficiency (SinESC achieves > 90% efficiency; trapezoidal ESCs typically achieve around 70%) allows for longer flight times and lower peak currents, which can increase battery longevity as well.

# Current-Sense Topology
Not just any ESC can perform sinusoidal control, and not all sinusoidal ESCs are created equal. Sinusoidal control requires that the ESC know the exact position of the rotor in order to properly align the magnetic field. Perfect alignment results in a perfect sine wave.

There are a few ways to find the position of the rotor:
- The first, most obvious method is to put a hall-effect (or similar) sensor on the motor to determine exactly where the rotor is. This sensor simply feeds data to the ESC. However, this is impractical for drones and wings due to durability, size, and weight concerns.
- The second way is to calculate the position of the rotor based on the currents flowing through each phase. Technically, measuring just the total motor current is sufficient, but this leads to poor position estimation and thus reduced performance. Alternatively, each half-bridge can connect to ground through a series current shunt and the voltage drop across each shunt can be amplified and read by an ADC in a microcontroller. This allows for the current through each phase to be known, resulting in far superior performance. Though more complicated, this is the current-sensing topology that SinESC employs.
- Finally, inline current-sensing is also possible. This technique involves placing shunts in series with only two of the three motor phases, since the third phase current can be calculated with Kirchoff's Current Law. However, this requires complicated, expensive amplifiers as high common-mode rejection ratio is critical. Some gate driver ICs integrate these amplifiers, but their gain and offset cannot be tuned to take full advantage of the ADC resolution. At high currents, ADC resolution is very valuable and should not be wasted. Thus, the second method was selected.

# Hardware Achievements
The 15 × 30mm SinESC Multi Edition PCB uses 0201-sized components when possible for reduced parasitic influences and reduced size. Maximum component density is approximately 50 components/cm². The microcontroller, gate driver, and current-sense amplifiers (all the control electronics) are implemented on the top layer, leaving the bottom layer for the power electronics. This provides room for wide traces. Multi Edition is designed in KiCAD and adheres to OSHPark design rules. All additional libraries are within the project files.

SinESC is designed specifically for efficiency and robustness:
- ST Microelectronics STM32F303C(Bx/Cx) microcontroller
- Maxim Integrated MAX4239 precision op-amps for current-sensing. Incredibly low input offset voltage of 0.1µV. Gain = 30V/V; offset tuned to use the full ADC resolution.
- Low-resistance current shunts (1mΩ) for reduced power loss
- Switching regulator provides logic-level voltage supply for control electronics (efficiency > 80%)
- Thermally optimized PCB layout
- Trinamic Motion Control TMC6100-LA gate driver
  - Fast switching slopes achieved with high gate drive current of 1.5A, decreasing switching losses
- Infineon Technologies BSC010N04LSI N-channel MOSFETs
  - Extremely low on-resistance (Rdson = 1.05mΩ) for reduced power loss
  - High drain-source voltage (Vds = 40V) for immunity to voltage spikes even at 6S Li-po voltages
- High-quality passive components
  - TDK ceramic capacitors
  - Panasonic metal film resistors (all 1% tolerance including shunts)

\*Many of these specifications will change; see the top of this page

# Versions
Regular Semantic Versioning is used for all software components of the project. As for PCB designs, Semantic Versioning has been modified. After Multi v2.3C, versions only reflect the PCB design since schematic changes directly influence the PCB. They are written like so:

vX.YZ where X and Y are numbers and Z is a letter. "v" is sometimes omitted.
- If a minor change is made that does NOT require a new stencil OR an updated BOM, Z is incremented.
- If a minor change requires a new stencil and/or an updated BOM, Y is incremented and Z is reset. Ideally, few minor changes are made between major releases.
- If a major change is made, X is incremented and Y and Z are reset.
