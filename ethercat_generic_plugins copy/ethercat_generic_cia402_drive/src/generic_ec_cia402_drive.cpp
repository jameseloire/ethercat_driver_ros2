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
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#include <numeric>
#include <cmath>

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace ethercat_generic_plugins
{

EcCiA402Drive::EcCiA402Drive()
: GenericEcSlave() {}
EcCiA402Drive::~EcCiA402Drive() {}

bool EcCiA402Drive::initialized() const {return initialized_;}

void EcCiA402Drive::processData(size_t index, uint8_t * domain_address)
{
  // Special case: ControlWord
  if (pdo_channels_info_[index].index == CiA402D_RPDO_CONTROLWORD) {
    if (is_operational_) {
      if (fault_reset_command_interface_index_ >= 0) {
        if (command_interface_ptr_->at(fault_reset_command_interface_index_) == 0) {
          last_fault_reset_command_ = false;
        }
        if (last_fault_reset_command_ == false &&
          command_interface_ptr_->at(fault_reset_command_interface_index_) != 0 &&
          !std::isnan(command_interface_ptr_->at(fault_reset_command_interface_index_)))
        {
          last_fault_reset_command_ = true;
          fault_reset_ = true;
        }
      }

      if (auto_state_transitions_) {
        pdo_channels_info_[index].default_value = transition(
          state_,
          pdo_channels_info_[index].ec_read(domain_address));
      }
    }
  }

  // Anti-kick for CSP:
  // before the drive is in CSP + Operation Enabled, keep target position on actual position
  // and ignore external position commands.
  if (pdo_channels_info_[index].index == CiA402D_RPDO_POSITION) {
    if (!std::isnan(last_position_)) {
      pdo_channels_info_[index].default_value =
        pdo_channels_info_[index].factor * last_position_ +
        pdo_channels_info_[index].offset;
    }
    const bool csp_enabled =
      (mode_of_operation_display_ == ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) &&
      (state_ == STATE_OPERATION_ENABLED);
    pdo_channels_info_[index].override_command = !csp_enabled;
  }

  // Anti-kick for CSV:
  // hold commanded velocity to 0 until CSV is confirmed active and drive is enabled.
  if (pdo_channels_info_[index].index == CiA402D_RPDO_VELOCITY) {
    pdo_channels_info_[index].default_value = 0.0;
    const bool csv_enabled =
      (mode_of_operation_display_ == ModeOfOperation::MODE_CYCLIC_SYNC_VELOCITY) &&
      (state_ == STATE_OPERATION_ENABLED);
    pdo_channels_info_[index].override_command = !csv_enabled;
  }

  // Anti-kick for CST:
  // hold commanded torque to 0 until CST is confirmed active and drive is enabled.
  if (pdo_channels_info_[index].index == CiA402D_RPDO_EFFORT) {
    pdo_channels_info_[index].default_value = 0.0;
    const bool cst_enabled =
      (mode_of_operation_display_ == ModeOfOperation::MODE_CYCLIC_SYNC_TORQUE) &&
      (state_ == STATE_OPERATION_ENABLED);
    pdo_channels_info_[index].override_command = !cst_enabled;
  }

  // setup mode of operation
  if (pdo_channels_info_[index].index == CiA402D_RPDO_MODE_OF_OPERATION) {
    if (mode_of_operation_ >= 0 && mode_of_operation_ <= 10) {
      pdo_channels_info_[index].default_value = mode_of_operation_;
    }
  }

  pdo_channels_info_[index].ec_update(domain_address);

  // get mode_of_operation_display_
  if (pdo_channels_info_[index].index == CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY) {
    mode_of_operation_display_ = pdo_channels_info_[index].last_value;
  }

  if (pdo_channels_info_[index].index == CiA402D_TPDO_POSITION) {
    last_position_ = pdo_channels_info_[index].last_value;
  }

  if (pdo_channels_info_[index].index == CiA402D_RPDO_POSITION && counter_ < 50) {
    std::cout << "[TRACE] cnt=" << counter_
              << " mode_disp=" << static_cast<int>(mode_of_operation_display_)
              << " status=" << status_word_
              << " pos_act=" << last_position_
              << " cmd_eff=" << pdo_channels_info_[index].last_value
              << " default=" << pdo_channels_info_[index].default_value
              << " override=" << (pdo_channels_info_[index].override_command ? 1 : 0)
              << std::endl;
  }

  // Special case: StatusWord
  if (pdo_channels_info_[index].index == CiA402D_TPDO_STATUSWORD) {
    status_word_ = pdo_channels_info_[index].last_value;
  }


  // CHECK FOR STATE CHANGE
  if (index == all_channels_.size() - 1) {  // if last entry  in domain
    if (status_word_ != last_status_word_) {
      state_ = deviceState(status_word_);
      if (state_ != last_state_) {
        std::cout << "STATE: " << DEVICE_STATE_STR.at(state_)
                  << " with status word :" << status_word_ << std::endl;
      }
    }
    initialized_ = ((state_ == STATE_OPERATION_ENABLED) &&
      (last_state_ == STATE_OPERATION_ENABLED)) ? true : false;

    last_status_word_ = status_word_;
    last_state_ = state_;
    counter_++;
  }
}

bool EcCiA402Drive::setupSlave(
  std::unordered_map<std::string, std::string> slave_paramters,
  std::vector<double> * state_interface,
  std::vector<double> * command_interface)
{
  state_interface_ptr_ = state_interface;
  command_interface_ptr_ = command_interface;
  paramters_ = slave_paramters;

  if (paramters_.find("slave_config") != paramters_.end()) {
    if (!setup_from_config_file(paramters_["slave_config"])) {
      return false;
    }
  } else {
    std::cerr << "EcCiA402Drive: failed to find 'slave_config' tag in URDF." << std::endl;
    return false;
  }

  setup_interface_mapping();
  setup_syncs();

  if (paramters_.find("mode_of_operation") != paramters_.end()) {
    mode_of_operation_ = std::stod(paramters_["mode_of_operation"]);
  }

  if (paramters_.find("command_interface/reset_fault") != paramters_.end()) {
    fault_reset_command_interface_index_ = std::stoi(paramters_["command_interface/reset_fault"]);
  }

  return true;
}

bool EcCiA402Drive::setup_from_config(YAML::Node drive_config)
{
  if (!GenericEcSlave::setup_from_config(drive_config)) {return false;}
  // additional configuration parameters for CiA402 Drives
  if (drive_config["auto_fault_reset"]) {
    auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
  }
  if (drive_config["auto_state_transitions"]) {
    auto_state_transitions_ = drive_config["auto_state_transitions"].as<bool>();
  }
  return true;
}

bool EcCiA402Drive::setup_from_config_file(std::string config_file)
{
  // Read drive configuration from YAML file
  try {
    slave_config_ = YAML::LoadFile(config_file);
  } catch (const YAML::ParserException & ex) {
    std::cerr << "EcCiA402Drive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  } catch (const YAML::BadFile & ex) {
    std::cerr << "EcCiA402Drive: failed to load drive configuration: " << ex.what() << std::endl;
    return false;
  }
  if (!setup_from_config(slave_config_)) {
    return false;
  }
  return true;
}

/** returns device state based upon the status_word */
DeviceState EcCiA402Drive::deviceState(uint16_t status_word)
{
  // MyActuator style decoding:
  // state is inferred from statusword bits directly (not from legacy CiA402 masks).
  const bool ready_to_switch_on = (status_word & (1u << SW_READY_TO_SWITCH_ON)) != 0u;
  const bool switched_on = (status_word & (1u << SW_SWITCHED_ON)) != 0u;
  const bool operation_enabled = (status_word & (1u << SW_OPERATION_ENABLED)) != 0u;
  const bool fault = (status_word & (1u << SW_FAULT)) != 0u;
  const bool quick_stop = (status_word & (1u << SW_QUICK_STOP)) != 0u;
  const bool switch_on_disabled = (status_word & (1u << SW_SWITCH_ON_DISABLED)) != 0u;

  if (fault) {
    return STATE_FAULT;
  }
  if (switch_on_disabled) {
    return STATE_SWITCH_ON_DISABLED;
  }
  if (ready_to_switch_on && switched_on && operation_enabled) {
    return quick_stop ? STATE_OPERATION_ENABLED : STATE_QUICK_STOP_ACTIVE;
  }
  if (ready_to_switch_on && switched_on) {
    return STATE_SWITCH_ON;
  }
  if (ready_to_switch_on) {
    return STATE_READY_TO_SWITCH_ON;
  }
  return STATE_UNDEFINED;
}

/** returns the control word that will take device from state to next desired state */
uint16_t EcCiA402Drive::transition(DeviceState state, uint16_t control_word)
{
  (void)control_word;
  // MyActuator enable sequence:
  // Shutdown (6) -> Switch on (7) -> Enable operation (15).
  constexpr uint16_t cw_shutdown = 0x0006;
  constexpr uint16_t cw_switch_on = 0x0007;
  constexpr uint16_t cw_enable_operation = 0x000F;
  constexpr uint16_t cw_fault_reset = 0x0080;

  switch (state) {
    case STATE_START:
      return cw_shutdown;
    case STATE_SWITCH_ON_DISABLED:
      return cw_shutdown;
    case STATE_READY_TO_SWITCH_ON:
      return cw_switch_on;
    case STATE_SWITCH_ON:
      return cw_enable_operation;
    case STATE_OPERATION_ENABLED:
      return cw_enable_operation;
    case STATE_QUICK_STOP_ACTIVE:
      return cw_enable_operation;
    case STATE_FAULT:
      if (auto_fault_reset_ || fault_reset_) {
        fault_reset_ = false;
        return cw_fault_reset;
      } else {
        return cw_shutdown;
      }
    default:
      break;
  }
  return cw_shutdown;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave)
