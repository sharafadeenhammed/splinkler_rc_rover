# 40-pin header usage ledger.  Every line marks what currently claims the pin:
#   USED = driven/read by this project — never re-use without unwiring it.
#   FREE = unclaimed, safe for new hardware.
# Physical pins 27/28 (BCM 0/1) are deliberately absent: they are the HAT
# EEPROM ID lines (ID_SD/ID_SC) and must stay reserved.
class PINOUT:
  # SAME AS SDA PIN FOR I2C.
  PIN3 = 2    # USED: I2C SDA  — MPU6050 IMU + HMC5883L compass (shared bus)
  # SAME AS SCL PIN FOR I2C.
  PIN5 = 3    # USED: I2C SCL  — MPU6050 IMU + HMC5883L compass (shared bus)
  # SAME AS PIN3.
  SDA = 2
  # SAME AS PIN5.
  SCL = 3
  PIN7 = 4    # USED: L298N ENA — left-motor speed PWM  (alias: ENA below)
  PIN8 = 14   # USED: UART TXD (/dev/serial0) — GPS transmit
  PIN10 = 15  # USED: UART RXD (/dev/serial0) — GPS receive
  PIN11 = 17  # USED: left-motor forward  → L298N IN1 (motor_controls)
  PIN12 = 18  # USED: left-motor reverse  → L298N IN2 (motor_controls)
  PIN13 = 27  # USED: right-motor forward → L298N IN3 (motor_controls)
  PIN15 = 22  # USED: right-motor reverse → L298N IN4 (motor_controls)
  PIN16 = 23  # USED: camera servo        (servo_controller.camera_servo)
  PIN18 = 24  # USED: soil-moisture sensor digital out (moisture_controller)
  PIN19 = 10  # FREE (alt function SPI MOSI — SPI is not used by this project)
  PIN21 = 9   # FREE (alt function SPI MISO — SPI is not used)
  PIN22 = 25  # USED: water pump transistor (pump_controller)
  PIN23 = 11  # FREE (alt function SPI SCLK — SPI is not used)
  PIN24 = 8   # FREE (alt function SPI CE0  — SPI is not used)
  PIN26 = 7   # FREE (alt function SPI CE1  — SPI is not used)
  PIN29 = 5   # USED: left encoder channel A  (encoder_controller)
  PIN31 = 6   # USED: ultrasonic TRIGGER      (ultrasonic_controller)
  PIN32 = 12  # USED: ultrasonic ECHO         (ultrasonic_controller)
  PIN33 = 13  # USED: left encoder channel B  (encoder_controller)
  PIN35 = 19  # USED: right encoder channel A (encoder_controller)
  PIN36 = 16  # USED: right encoder channel B (encoder_controller)
  PIN37 = 26  # USED: sensor-sweep servo      (servo_controller.sensor_servo)
  PIN38 = 20  # USED: L298N ENB — right-motor speed PWM (alias: ENB below)
  PIN40 = 21  # USED: DHT11 data line (kernel dtoverlay=dht11,gpiopin=21)

  # L298N enable (speed) pins — added for the L298N driver swap.  Speed is
  # PWM'd HERE while the four IN pins only pick direction.  The ENA/ENB
  # jumper caps on the driver board must be REMOVED, otherwise the enables
  # are tied HIGH and the motors run at full speed regardless of software.
  ENA = PIN19   # left-motor speed PWM  (BCM 4)
  ENB = PIN38  # right-motor speed PWM (BCM 20)

  # JGB37-520 encoder outputs (BCM numbering).
  # Yellow = channel A, green = channel B.
  RIGHT_ENCODER_A = PIN29
  RIGHT_ENCODER_B = PIN33
  LEFT_ENCODER_A = PIN35
  LEFT_ENCODER_B = PIN36


__all__ = ["PINOUT"]
