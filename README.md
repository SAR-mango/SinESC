# Overview
SinESC brings sinusoidal motor control to the RC drone & wing communities while retaining standard features and low cost.

There are two versions of SinESC. Wing Edition and Multi Edition:

Wing Edition supports CAN and standard PWM control signals. CAN is intended for Pixhawk users, and standard PWM is for direct connection to an RC receiver.

Multi Edition communicates with a flight controller though SinWire, a custom 1-wire protocol that enables configuration, firmware updates, and speed control all through one wire.
SinWire support will be added to Betaflight soon. It aims to match or exceed the update rate of DSHOT600.
# Benefits of Sinusoidal Control—Efficiency, Efficiency, Efficiency
Brushless DC motors work best when their phase voltage waveforms are sinusoid-shaped. Try it yourself—connect a BLDC motor to an oscilloscope and spin the rotor with your fingers. You will see a sine wave. 

Regular BLHELI_32 ESCs employ a simple motor control method known as "trapezoidal drive," in which the phase voltage waveforms are trapezoid-shaped rather than sinusoid-shaped. This is non-ideal as the fields are not always perfectly aligned with the rotor, which means that torque is not constantly maximized at a given power. This causes the rotor to effectively "jerk in a circle" rather than spin continuously, resulting in lower efficiency and rougher flight. Furthermore, the sharp edges of the trapezoidal signal also introduce high-frequency noise which contributes to the above disadvantages as well. This also makes the motors run louder and hotter.

Sinusoidal control minimizes torque ripple, maximizing efficiency. The magnetic fields are aligned precisely to exert the maximum torque force on the rotor at any given position. Minimized torque ripple leads to drastically smoother flight. Greater efficiency (SinESC achieves > 97% efficiency; trapezoidal ESCs typically achieve 70%) allows for longer flight times and lower peak currents, which can increase battery longevity as well.

SinESC is designed specifically for efficiency—circuit topologies have been selected with efficiency as the top priority:

- Low-resistance current shunts... 1mΩ (discussed below)
- Switching regulator provides a logic-level voltage supply for all control electronics
- Extremely low on-resistance MOSFETs selected for the power stage... Rdson = 1.05mΩ
- Fast switching slopes achieved with high gate-drive current, minimizing switching losses

# Hardware Achievements
Not just any ESC can perform sinusoidal control, and not all sinusoidal ESCs are created equal. Sinusoidal control requires the ESC to know the exact position of the rotor in order to properly align the magnetic field with the rotor. Perfect alignment means a perfect sine wave.

There are a few ways to find the position of the rotor:

The first, most obvious method is to put a sensor on the motor that uses the hall effect or something similar to determine exactly where the rotor is. This sensor simply feeds data to the ESC. However, this is impractical for drones and wings due to durability, size, and weight concerns.

The second way is to calculate the position of the rotor based on the currents flowing through each phase
