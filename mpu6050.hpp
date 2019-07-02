#ifndef MPU6050_HPP
#define MPU6050_HPP

#include "hwlib.hpp"
#include <math.h>


class mpu6050 {
private:
  hwlib::i2c_bus bus;
  u_int8_t adress;

  int16_t readRawAcceleX() {
    uint8_t val_H[1] = {0x3B};
    bus.write(adress).write(val_H, 1);
    bus.read(adress).read(val_H, 1);

    uint8_t val_L[1] = {0x3C};
    bus.write(adress).write(val_L, 1);
    bus.read(adress).read(val_L, 1);

    int16_t ret = 0;
    ret = (val_H[0] << 8) + val_L[0];
    return ret;
  }

  int16_t readRawAcceleY() {
    uint8_t val_H[1] = {0x3D};
    bus.write(adress).write(val_H, 1);
    bus.read(adress).read(val_H, 1);

    uint8_t val_L[1] = {0x3E};
    bus.write(adress).write(val_L, 1);
    bus.read(adress).read(val_L, 1);

    int16_t ret = 0;
    ret = (val_H[0] << 8) + val_L[0];
    return ret;
  }

  int16_t readRawAcceleZ() {
    uint8_t val_H[1] = {0x3F};
    bus.write(adress).write(val_H, 1);
    bus.read(adress).read(val_H, 1);

    uint8_t val_L[1] = {0x40};
    bus.write(adress).write(val_L, 1);
    bus.read(adress).read(val_L, 1);

    int16_t ret = 0;
    ret = (val_H[0] << 8) + val_L[0];
    return ret;
  }

  int16_t readRawGyroX() {
    uint8_t val_H[1] = {0x43};
    bus.write(adress).write(val_H, 1);
    bus.read(adress).read(val_H, 1);

    uint8_t val_L[1] = {0x44};
    bus.write(adress).write(val_L, 1);
    bus.read(adress).read(val_L, 1);

    int16_t ret = 0;
    ret = (val_H[0] << 8) + val_L[0];
    return ret;
  }

  int16_t readRawGyroY() {
    uint8_t val_H[1] = {0x45};
    bus.write(adress).write(val_H, 1);
    bus.read(adress).read(val_H, 1);

    uint8_t val_L[1] = {0x46};
    bus.write(adress).write(val_L, 1);
    bus.read(adress).read(val_L, 1);

    int16_t ret = 0;
    ret = (val_H[0] << 8) + val_L[0];
    return ret;
  }

  int16_t readRawGyroZ() {
    uint8_t val_H[1] = {0x47};
    bus.write(adress).write(val_H, 1);
    bus.read(adress).read(val_H, 1);

    uint8_t val_L[1] = {0x48};
    bus.write(adress).write(val_L, 1);
    bus.read(adress).read(val_L, 1);

    int16_t ret = 0;
    ret = (val_H[0] << 8) + val_L[0];
    return ret;
  }

public:
  mpu6050(hwlib::i2c_bus &bus, const u_int8_t &adress)
      : bus(bus), adress(adress) {}

  void init() {
    const uint8_t start[2] = {0x6B, 0x00};
    bus.write(adress).write(start, 2);
    hwlib::wait_ms(20);
    hwlib::cout << "MPU initialized" << hwlib::endl;
  }

  float readTemp() {
    uint8_t val_H[1] = {0x41};
    bus.write(adress).write(val_H, 1);
    bus.read(adress).read(val_H, 1);

    uint8_t val_L[1] = {0x42};
    bus.write(adress).write(val_L, 1);
    bus.read(adress).read(val_L, 1);

    int16_t ret = 0;
    ret = (val_H[0] << 8) + val_L[0];
    float celcius = (ret / 340) + 32.5;
    return celcius;
  }

  int16_t readRollAngle() { // y

    int_fast16_t xa = readRawAcceleX();
    int_fast16_t ya = readRawAcceleY();
    int_fast16_t za = readRawAcceleZ();
    int_fast16_t rollAngle =
        (atan(ya / sqrt((xa * xa) + (za * za))) * 180 / 3.14156) - 0.59;
    return rollAngle;
  }

  int16_t readPitchAngle() { // x
    int_fast16_t xa = readRawAcceleX();
    int_fast16_t ya = readRawAcceleY();
    int_fast16_t za = readRawAcceleZ();
    int_fast16_t pitchAngle =
        (atan(xa / sqrt((ya * ya) + (za * za))) * 180 / 3.14156) - 0.59;
    return pitchAngle;
  }
};
#endif // MPU6050_HPP