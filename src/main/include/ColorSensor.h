#pragma once

#include <frc/I2C.h>

namespace frc {

class ColorSensor : public I2C {
 public:
  explicit ColorSensor(Port port, int deviceAddress);
  int getRed();
  int getGreen();
  int getBlue();
  int getWhite();
  int getClear();
  int getProximity();
  void setGain(int gain);
  ColorSensor(ColorSensor&&) = default;
  ColorSensor& operator=(ColorSensor&&) = default;
};

}  // namespace frc
