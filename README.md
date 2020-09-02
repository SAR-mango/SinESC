# Overview
SinESC brings Sinusoidal Motor Control to the FPV community without giving up standard features or low cost.

There are two versions of SinESC. Wing Edition and Multi Edition:

Wing Edition supports CAN and standard PWM control signals. CAN is intended for Pixhawk users, and standard PWM is intended to connect straight to an RC receiver.

Multi Edition supports SinWire, a custom 1-wire protocol that enables configuration, firmware updates, and speed control all through one wire.
SinWire support will be added to Betaflight soon. It aims to match or exceed the update rate of DSHOT600.
# Benefits of Sinusoidal Control
Brushless DC motors work best when their phase voltage waveforms are sinusoid-shaped. Try it yourself... connect a BLDC motor to an oscilloscope and spin the rotor with your fingers. You will see a sine wave. 

Sinusoidal control minimizes torque ripple and maximizes efficiency, as the magnetic fields are perfectly aligned with the rotor. Minimized torque ripple can lead to drastically smoother, quieter flight. Greater efficiency (SinESC achieves > 97% efficiency while most trapezoidal ESCs only achieve around 70% efficiency) allows for longer flight times and lower peak currents, which can increase battery longevity as well.

Regular BLHELI_32 ESCs employ a simple motor control method known as "trapezoidal drive," in which the phase voltage waveforms are trapezoid-shaped rather than sinusoid-shaped. This is non-ideal as the fields are not perfectly aligned with the rotor, causing tiny jerks while the motor spins, The sharp edges of the signal also introduce high-frequency noise. The tiny jerks and added noise decrease efficiency, torque, and smoothness. Trapezoidally driven motors are louder as well.
