#ifndef LSM6DSR_RPL4_LIB_LSM6DSR_H
#define LSM6DSR_RPL4_LIB_LSM6DSR_H

#include "lsm6dsr_rpl4_lib/registers.h"
#include "rpl4/peripheral/spi.hpp"
#include "rpl4/rpl4.hpp"

namespace lsm6dsr_rpl4_lib {
class LSM6DSR {
 public:
  LSM6DSR(std::shared_ptr<rpl::Spi> spi, rpl::Spi::ChipSelect cs);
  ~LSM6DSR() = default;

  bool Init();
  bool ReadAccel(float* x, float* y, float* z);
  bool ReadGyro(float* x, float* y, float* z);

  bool WriteRegister(Register address, uint8_t* data);
  bool ReadRegister(Register address, uint8_t* data);

  bool WriteRegisters(Register start_address, uint8_t* data, size_t length);
  bool ReadRegisters(Register start_address, uint8_t* data, size_t length);

  uint8_t WhoAmI();

  enum class EnableState : bool {
    kEnabled = true,
    kDisabled = false,
  };

  bool WriteI3cEnabled(EnableState enable_state);
  bool WriteAutoIncrementEnabled(EnableState enable_state);

  enum class GyroDataRate : uint8_t {
    kPowerDown = 0b0000,
    k12_5Hz = 0b0001,
    k26Hz = 0b0010,
    k52Hz = 0b0011,
    k104Hz = 0b0100,
    k208Hz = 0b0101,
    k416Hz = 0b0110,
    k833Hz = 0b0111,
    k1660Hz = 0b1000,
    k3330Hz = 0b1001,
    k6660Hz = 0b1010,
  };

  bool WriteGyroDataRate(GyroDataRate data_rate);

  enum class GyroFullScale : uint8_t {
    k125dps = 0b0010,
    k250dps = 0b0000,
    k500dps = 0b0100,
    k1000dps = 0b1000,
    k2000dps = 0b1100,
    k4000dps = 0b0001,
  };

  bool WriteGyroFullScale(GyroFullScale full_scale);
  GyroFullScale ReadGyroFullScale();
  void ConfigureGyroSensitivity(GyroFullScale full_scale);

  enum class AccelDataRate : uint8_t {
    kPowerDown = 0b0000,
    k1_6Hz = 0b1011,
    k12_5Hz = 0b0001,
    k26Hz = 0b0010,
    k52Hz = 0b0011,
    k104Hz = 0b0100,
    k208Hz = 0b0101,
    k416Hz = 0b0110,
    k833Hz = 0b0111,
    k1660Hz = 0b1000,
    k3330Hz = 0b1001,
    k6660Hz = 0b1010,
  };

  bool WriteAccDataRate(AccelDataRate data_rate);

  enum class AccelFullScale : uint8_t {
    k2g = 0b00,
    k4g = 0b10,
    k8g = 0b11,
    k16g = 0b01,
  };

  bool WriteAccFullScale(AccelFullScale full_scale);
  AccelFullScale ReadAccFullScale();
  void ConfigureAccSensitivity(AccelFullScale full_scale);

  bool ReadAccAndGyro(float& gyro_x, float& gyro_y, float& gyro_z,
                      float& acc_x, float& acc_y, float& acc_z);

 private:
  std::shared_ptr<rpl::Spi> spi_;
  rpl::Spi::ChipSelect cs_;
  float acc_sensitivity_ = 0.0f;
  float gyro_sensitivity_ = 0.0f;
};

}  // namespace lsm6dsr_rpl4_lib

#endif  // LSM6DSR_RPL4_LIB_LSM6DSR_H