class PINOUT:
  # SAME AS SDA PIN FOR I2C.
  PIN3 = 2
  # SAME AS SCL PIN FOR I2C.
  PIN5 = 3
  # SAME AS PIN3.
  SDA = 2
  # SAME AS PIN5.
  SCL = 3
  PIN7 = 4
  PIN8 = 14
  PIN10 = 15
  PIN11 = 17
  PIN12 = 18
  PIN13 = 27
  PIN15 = 22
  PIN16 = 23
  PIN18 = 24
  PIN19 = 10
  PIN21 = 9
  PIN22 = 25
  PIN23 = 11
  PIN24 = 8
  PIN26 = 7
  PIN29 = 5
  PIN31 = 6
  PIN32 = 12
  PIN33 = 13
  PIN35 = 19
  PIN36 = 16
  PIN37 = 26
  PIN38 = 20
  PIN40 = 21

  # JGB37-520 encoder outputs (BCM numbering).
  # Yellow = channel A, green = channel B.
  LEFT_ENCODER_A = PIN29
  LEFT_ENCODER_B = PIN33
  RIGHT_ENCODER_A = PIN35
  RIGHT_ENCODER_B = PIN36


__all__ = ["PINOUT"]
