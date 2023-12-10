# Lemon Pepper Stepper Driver
Small, all in one hybrid stepper + BLDC driver, designed to mount to the back of NEMA 14/17 size stepper motors. The board supports up to 48V @ 1.5A with a choice of either step-dir, CAN bus, I2C, UART or USB input. Additionally, there are 5 GPIO broken out that can be used for a second encoder (AB hardware encoder or I2C), endstops or other general purposes. Current sensing is also included on-board.
The boards can easily be assembled by JLC and the design has been cost-optimized, coming in at about $20 per board fully assembled (minus connectors, which need to be modified to sit flat on motor back). 

Due to the high pole pair count of stepper motors, a high resolution (21 bit) magnetic encoder is used, supporting both SPI and hardware ABI position encoding. 
Despite the rather high theoretical performance of this board I think you would need some serious cooling to actually hit the potential 50-70W specs of the parts. A heatsink has been designed and produced that works well with the board, giving a uniform back profile and small fins for heat dissipation.
![Photo of PCB](/many.jpeg)
![PCB on motor](/motor.png)
![Heatsink on motor](/heatsink.jpeg)
