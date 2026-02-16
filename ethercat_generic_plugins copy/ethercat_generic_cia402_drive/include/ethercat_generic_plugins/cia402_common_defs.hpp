// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ETHERCAT_GENERIC_PLUGINS__CIA402_COMMON_DEFS_HPP_
#define ETHERCAT_GENERIC_PLUGINS__CIA402_COMMON_DEFS_HPP_

#include <cstdint>
#include <map>
#include <string>

#define CiA402D_RPDO_CONTROLWORD  ((uint16_t) 0x6040)
#define CiA402D_RPDO_POSITION  ((uint16_t) 0x607a)
#define CiA402D_RPDO_VELOCITY  ((uint16_t) 0x60ff)
#define CiA402D_RPDO_EFFORT  ((uint16_t) 0x6071)
#define CiA402D_RPDO_MODE_OF_OPERATION  ((uint16_t) 0x6060)

#define CiA402D_TPDO_POSITION ((uint16_t) 0x6064)
#define CiA402D_TPDO_STATUSWORD  ((uint16_t) 0x6041)
#define CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY  ((uint16_t) 0x6061)

enum DeviceState
{
  STATE_START = 0,
  STATE_SWITCH_ON_DISABLED,
  STATE_READY_TO_SWITCH_ON,
  STATE_SWITCH_ON,
  STATE_OPERATION_ENABLED,
  STATE_QUICK_STOP_ACTIVE,
  STATE_FAULT,
  STATE_UNDEFINED
};

// MyActuator status word bit positions from vendor documentation.
enum StatusWordBit : uint8_t
{
  SW_READY_TO_SWITCH_ON = 0,
  SW_SWITCHED_ON = 1,
  SW_OPERATION_ENABLED = 2,
  SW_FAULT = 3,
  SW_VOLTAGE_ENABLED = 4,
  SW_QUICK_STOP = 5,
  SW_SWITCH_ON_DISABLED = 6,
  SW_WARNING = 7,
  SW_REMOTE = 9,
  SW_TARGET_REACHED = 10,
  SW_INTERNAL_LIMIT_ACTIVE = 11,
  SW_FOLLOW_TARGET_POSITION = 12,
  SW_FOLLOW_TARGET_ERROR_ALARM = 13
};

enum ModeOfOperation
{
  MODE_NO_MODE = 0,
  MODE_CYCLIC_SYNC_POSITION = 8,
  MODE_CYCLIC_SYNC_VELOCITY = 9,
  MODE_CYCLIC_SYNC_TORQUE = 10
};

enum ControlWordBits : uint8_t
{
  CW_SWITCH_ON = 0,
  CW_ENABLE_VOLTAGE = 1,
  CW_QUICK_STOP = 2,
  CW_ENABLE_OPERATION = 3
};

const std::map<DeviceState, std::string> DEVICE_STATE_STR = {
  {STATE_START, "Start"},
  {STATE_SWITCH_ON_DISABLED, "Switch On Disabled"},
  {STATE_READY_TO_SWITCH_ON, "Ready To Switch On"},
  {STATE_SWITCH_ON, "Switched On"},
  {STATE_OPERATION_ENABLED, "Operation Enabled"},
  {STATE_QUICK_STOP_ACTIVE, "Quick Stop Active"},
  {STATE_FAULT, "Fault"},
  {STATE_UNDEFINED, "Undefined"}
};

#endif  // ETHERCAT_GENERIC_PLUGINS__CIA402_COMMON_DEFS_HPP_
