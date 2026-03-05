#pragma once
#include <string>

/// CamelCase → snake_case 変換
/// 例: BLDCMotor → bldc_motor, IMUSensor → imu_sensor, SetEnable → set_enable
std::string camel_to_snake(const std::string & name);
