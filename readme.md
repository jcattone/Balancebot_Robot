Pinout:
  I2C SDA: 21
  I2C SCL: 22
Devices
  2S-18650: Motor power
    Diode protecting DRV8833
  MP1584 buck converter (5V out for MCU)
  MPU-6050: I2C
    (optional) 22uF cap across power
  DRV8833
    Motor 1: Pins 32, 33
    Motor 2: Pins 25, 26
  AS5600
    SDA/SCL/3V3/GND
    DIR pulled to GND
  10k pull-up resistors on both I2C lines
  1000uF cap on motor power
  0.1uF bypass cap on each motor power

  