# Lemon Pepper Stepper Driver
Small, all in one hybrid stepper + BLDC driver. The board supports up to 48V @ 1.5A with a choice of either step-dir or CANbus inputs.
The boards can easily be assembled by JLC and the design has been cost-optimized, coming in at about $20 per board fully assembled. 

Due to the high pole pair count of stepper motors, a high resolution (21 bit) magnetic encoder is used, supporting both SPI and hardware ABI position encoding. 
Despite the rather high theoretical performance of this board I think you would need some serious cooling to actually hit the potential 50-70W specs of the parts.

![Render of PCB](/render.jpg)
