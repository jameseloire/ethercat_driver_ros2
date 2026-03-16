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

  // setup current position as default position
  if (pdo_channels_info_[index].index == CiA402D_RPDO_POSITION) {
    if (mode_of_operation_display_ != ModeOfOperation::MODE_NO_MODE) {
      pdo_channels_info_[index].default_value =
        pdo_channels_info_[index].factor * last_position_ +
        pdo_channels_info_[index].offset;
    }
    pdo_channels_info_[index].override_command =
      (mode_of_operation_display_ != ModeOfOperation::MODE_CYCLIC_SYNC_POSITION) ? true : false;
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
    if (!is_operational_) {
      status_word_ = 0;
      pdo_channels_info_[index].last_value = 0;
    } else {
      status_word_ = pdo_channels_info_[index].last_value;
    }
  }

  if (index == all_channels_.size() - 1) { // if last entry in domain
    // Mirror motion command interfaces into dedicated state interfaces for monitoring.
    if (target_position_state_interface_index_ >= 0 && position_command_interface_index_ >= 0) {
      state_interface_ptr_->at(target_position_state_interface_index_) =
        command_interface_ptr_->at(position_command_interface_index_);
    }
    if (target_velocity_state_interface_index_ >= 0 && velocity_command_interface_index_ >= 0) {
      state_interface_ptr_->at(target_velocity_state_interface_index_) =
        command_interface_ptr_->at(velocity_command_interface_index_);
    }
    if (target_effort_state_interface_index_ >= 0 && effort_command_interface_index_ >= 0) {
      state_interface_ptr_->at(target_effort_state_interface_index_) =
        command_interface_ptr_->at(effort_command_interface_index_);
    }
    if (target_max_torque_state_interface_index_ >= 0 && max_torque_command_interface_index_ >= 0) {
      state_interface_ptr_->at(target_max_torque_state_interface_index_) =
        command_interface_ptr_->at(max_torque_command_interface_index_);
    }

    // CHECK FOR STATE CHANGE
    if (status_word_ != last_status_word_) {
      state_ = deviceState(status_word_);
      if (state_ != last_state_) {
        std::cout << "STATE -- : " << DEVICE_STATE_STR.at(state_)
                  << " with status word -- :" << status_word_
                  << "[TRACE] cnt=" << counter_
                  << " pos_act=" << last_position_
                  << " cmd_eff=" << pdo_channels_info_[index].last_value
                  << " default=" << pdo_channels_info_[index].default_value
                  << " override=" << (pdo_channels_info_[index].override_command ? 1 : 0)
                  << std::endl;
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
  if (paramters_.find("command_interface/position") != paramters_.end()) {
    position_command_interface_index_ = std::stoi(paramters_["command_interface/position"]);
  }
  if (paramters_.find("command_interface/velocity") != paramters_.end()) {
    velocity_command_interface_index_ = std::stoi(paramters_["command_interface/velocity"]);
  }
  if (paramters_.find("command_interface/effort") != paramters_.end()) {
    effort_command_interface_index_ = std::stoi(paramters_["command_interface/effort"]);
  }
  if (paramters_.find("command_interface/max_torque") != paramters_.end()) {
    max_torque_command_interface_index_ = std::stoi(paramters_["command_interface/max_torque"]);
  }

  if (paramters_.find("state_interface/target_position") != paramters_.end()) {
    target_position_state_interface_index_ = std::stoi(paramters_["state_interface/target_position"]);
  }
  if (paramters_.find("state_interface/target_velocity") != paramters_.end()) {
    target_velocity_state_interface_index_ = std::stoi(paramters_["state_interface/target_velocity"]);
  }
  if (paramters_.find("state_interface/target_effort") != paramters_.end()) {
    target_effort_state_interface_index_ = std::stoi(paramters_["state_interface/target_effort"]);
  }
  if (paramters_.find("state_interface/target_max_torque") != paramters_.end()) {
    target_max_torque_state_interface_index_ = std::stoi(paramters_["state_interface/target_max_torque"]);
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
  if ((status_word & 0b01001111) == 0b00000000) {
    return STATE_NOT_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01001111) == 0b01000000) {
    return STATE_SWITCH_ON_DISABLED;
  } else if ((status_word & 0b01101111) == 0b00100001) {
    return STATE_READY_TO_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100011) {
    return STATE_SWITCH_ON;
  } else if ((status_word & 0b01101111) == 0b00100111) {
    return STATE_OPERATION_ENABLED;
  } else if ((status_word & 0b01101111) == 0b00000111) {
    return STATE_QUICK_STOP_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001111) {
    return STATE_FAULT_REACTION_ACTIVE;
  } else if ((status_word & 0b01001111) == 0b00001000) {
    return STATE_FAULT;
  }
  return STATE_UNDEFINED;
}

/** returns the control word that will take device from state to next desired state */
uint16_t EcCiA402Drive::transition(DeviceState state, uint16_t control_word)
{
  switch (state) {
    case STATE_START:                     // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON:    // -> STATE_SWITCH_ON_DISABLED (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED:        // -> STATE_READY_TO_SWITCH_ON
      return (control_word & 0b01111110) | 0b00000110;
    case STATE_READY_TO_SWITCH_ON:        // -> STATE_SWITCH_ON
      return (control_word & 0b01110111) | 0b00000111;
    case STATE_SWITCH_ON:                 // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_OPERATION_ENABLED:         // -> GOOD
      return control_word;
    case STATE_QUICK_STOP_ACTIVE:         // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_FAULT_REACTION_ACTIVE:     // -> STATE_FAULT (automatic)
      return control_word;
    case STATE_FAULT:                     // -> STATE_SWITCH_ON_DISABLED
      if (auto_fault_reset_ || fault_reset_) {
        fault_reset_ = false;
        return (control_word & 0b11111111) | 0b10000000;     // automatic reset
      } else {
        return control_word;
      }
    default:
      break;
  }
  return control_word;
}

}  // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave)
