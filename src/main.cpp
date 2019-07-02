#include "mpu6050.hpp"



int main() {

  auto scl = hwlib::target::pin_oc(
      hwlib::target::pins::d21); /*!< Defines the clock pin for the i2c bus */
  auto sda = hwlib::target::pin_oc(
      hwlib::target::pins::d20); /*!< Defines the Data pin for the i2c bus */
  auto iic = hwlib::i2c_bus_bit_banged_scl_sda(
      scl,
      sda); /*!< Initializes i2c bus  using the Bit Bangged implementation*/
  auto chip = mpu6050(iic, 0x68); /*!< Creates a mpu6050 type object with the
                                     i2c bus, and the adress  0x68*/

  chip.init(); /*!< Calls the initialization method of the mpu6050 */
  for (;;) {

    int Pitch = chip.readPitchAngle();
    int Roll = chip.readRollAngle();
    int temperature = chip.readTemp();

    hwlib::cout << "   The Pitch Angle is:  " << Pitch << hwlib::endl;
    hwlib::cout << "   The Roll Angle  is:   " << Roll << hwlib::endl;
    hwlib::cout << "   The Temperature is:  " << temperature << hwlib::endl;
    hwlib::wait_ms(500);
  }
}
