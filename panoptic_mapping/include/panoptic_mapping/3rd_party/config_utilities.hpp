/**
AUTHOR:       Lukas Schmid <schmluk@mavt.ethz.ch>
AFFILIATION:  Autonomous Systems Lab (ASL), ETH Zürich
SOURCE:       https://github.com/ethz-asl/config_utilities
VERSION:      1.1.4
LICENSE:      BSD-3-Clause

Copyright 2020 Autonomous Systems Lab (ASL), ETH Zürich.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Raises a redefined warning if different versions are used. v=MMmmPP.
#define CONFIG_UTILITIES_VERSION 010104

/**
 * Depending on which headers are available, ROS dependencies are included in
 * the library. Make sure to include config_utilities.hpp after these headers.
 */

// <ros/node_handle.h>
#ifdef ROSCPP_NODE_HANDLE_H
#ifndef CONFIG_UTILITIES_ROS_ENABLED
#define CONFIG_UTILITIES_ROS_ENABLED
#endif  // CONFIG_UTILITIES_ROS_ENABLED
#endif  // ROSCPP_NODE_HANDLE_H

// <kindr/minimal/quat-transformation.h>
#ifdef KINDR_MINIMAL_QUAT_TRANSFORMATION_H_
#ifndef CONFIG_UTILITIES_TRANSFORMATION_ENABLED
#define CONFIG_UTILITIES_TRANSFORMATION_ENABLED
#endif  // CONFIG_UTILITIES_TRANSFORMATION_ENABLED
#endif  // KINDR_MINIMAL_QUAT_TRANSFORMATION_H_

#ifndef CONFIG_UTILITIES_CORE_HPP_
#define CONFIG_UTILITIES_CORE_HPP_

#include <algorithm>
#include <cstring>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <unordered_map>
#include <utility>

#include <glog/logging.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace config_utilities {

/**
 * ==================== Settings ====================
 */
namespace internal {
struct GlobalSettings {
  GlobalSettings(const GlobalSettings& other) = delete;
  GlobalSettings& operator=(const GlobalSettings& other) = delete;

  // Settings
  unsigned int default_print_width = 80;
  unsigned int default_print_indent = 30;
  unsigned int default_subconfig_indent = 3;

  static GlobalSettings& instance() {
    static GlobalSettings settings;
    return settings;
  }

 private:
  GlobalSettings() = default;
};
}  // namespace internal

// Access.
inline internal::GlobalSettings& GlobalSettings() {
  return internal::GlobalSettings::instance();
}

/**
 * ==================== Utilities ====================
 */

// Add required command line arguments if needed. Keeps memory over the scope
// of existence for this class.
class RequiredArguments {
 public:
  explicit RequiredArguments(int* argc, char*** argv,
                             const std::vector<std::string>& arguments) {
    // Read old arguments to string.
    std::vector<std::string> old_args;
    old_args.reserve(*argc);
    for (int i = 0; i < *argc; ++i) {
      old_args.emplace_back((*argv)[i]);
    }

    // Detect new arguments.
    std::vector<std::string> added_args;
    for (const std::string& arg : arguments) {
      if (std::find(old_args.begin(), old_args.end(), arg) == old_args.end()) {
        added_args.push_back(arg);
      }
    }

    // Write old arguments.
    *argc = old_args.size() + added_args.size();
    argv_aux_ = std::vector<std::unique_ptr<char>>(*argc);
    for (int i = 0; i < old_args.size(); ++i) {
      argv_aux_[i].reset(new char[std::strlen((*argv)[i]) + 1]);
      strcpy(argv_aux_[i].get(), (*argv)[i]);
    }

    // Write new arguments.
    for (int i = old_args.size(); i < *argc; ++i) {
      argv_aux_[i].reset(
          new char[std::strlen(added_args[i - old_args.size()].c_str()) +
                   1]);  // Extra char for null-terminated string.
      strcpy(argv_aux_[i].get(), added_args[i - old_args.size()].c_str());
    }

    // Write argv.
    argv_.reserve(*argc);
    for (int i = 0; i < *argc; ++i) {
      argv_[i] = argv_aux_[i].get();
    }
    *argv = argv_.data();
  }

 private:
  std::vector<char*> argv_;
  std::vector<std::unique_ptr<char>> argv_aux_;
};

/**
 * ==================== Internal Utilities ====================
 */
namespace internal {
// Printing utility
inline std::string printCenter(const std::string& text, int width,
                               char symbol) {
  int first = std::max((width - static_cast<int>(text.length()) - 2) / 2, 0);
  std::string result = std::string(first, symbol) + " " + text + " ";
  result += std::string(std::max(width - static_cast<int>(result.length()), 0),
                        symbol);
  return result;
}

// Type verification
struct ConfigInternalVerificator {};
template <typename T>
inline bool isConfig(const T* candidate) {
  try {
    throw candidate;
  } catch (const ConfigInternalVerificator*) {
    return true;
  } catch (...) {
  }
  return false;
}

// Setup param type. These contain all params of the nodehandle and the full
// namespaces, the params "_name_space" and "_name_space_private" are reserved
// for the target namespace to look up data. Namespacing adheres to ROS
// standards with "/" for global and "~" for private namespaces.
using ParamMap = std::unordered_map<std::string, XmlRpc::XmlRpcValue>;

// XML casts. Default is not castable.
template <typename T>
inline bool xmlCast(XmlRpc::XmlRpcValue xml, T*) {
  return false;
}

// NOTE: the copy of the xml values is required for compatibility with ROS
// Kinetic, since the cast to value type is a non-const operation.

inline bool xmlCast(XmlRpc::XmlRpcValue xml, bool* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean: {
      if (param) {
        *param = static_cast<bool>(xml);
      }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeInt: {
      if (param) {
        *param = static_cast<bool>(static_cast<int>(xml));
      }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeDouble: {
      if (param) {
        *param = static_cast<bool>(static_cast<double>(xml));
      }
      return true;
    }
    default:
      return false;
  }
}

inline bool xmlCast(XmlRpc::XmlRpcValue xml, int* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean: {
      if (param) {
        *param = static_cast<int>(static_cast<bool>(xml));
      }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeInt: {
      if (param) {
        *param = static_cast<int>(xml);
      }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeDouble: {
      if (param) {
        *param = static_cast<int>(static_cast<double>(xml));
      }
      return true;
    }
    default:
      return false;
  }
}

inline bool xmlCast(XmlRpc::XmlRpcValue xml, float* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean: {
      if (param) {
        *param = static_cast<float>(static_cast<bool>(xml));
      }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeInt: {
      if (param) {
        *param = static_cast<float>(static_cast<int>(xml));
      }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeDouble: {
      if (param) {
        *param = static_cast<float>(static_cast<double>(xml));
      }
      return true;
    }
    default:
      return false;
  }
}

inline bool xmlCast(XmlRpc::XmlRpcValue xml, double* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeBoolean: {
      if (param) {
        *param = static_cast<double>(static_cast<bool>(xml));
      }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeInt: {
      if (param) {
        *param = static_cast<double>(static_cast<int>(xml));
      }
      return true;
    }
    case XmlRpc::XmlRpcValue::Type::TypeDouble: {
      if (param) {
        *param = static_cast<double>(xml);
      }
      return true;
    }
    default:
      return false;
  }
}

inline bool xmlCast(XmlRpc::XmlRpcValue xml, std::string* param = nullptr) {
  switch (xml.getType()) {
    case XmlRpc::XmlRpcValue::Type::TypeString: {
      if (param) {
        *param = static_cast<std::string>(xml);
      }
      return true;
    }
    default:
      return false;
  }
}

template <typename T>
inline bool xmlCast(XmlRpc::XmlRpcValue xml, std::vector<T>* param = nullptr) {
  if (xml.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    return false;
  }
  if (param) {
    param->resize(xml.size());
    for (int i = 0; i < xml.size(); i++) {
      T value;
      if (!xmlCast(xml[i], &value)) {
        return false;
      }
      param->at(i) = value;
    }
  }
  return true;
}

struct ConfigInternal;
}  // namespace internal

/**
 * ==================== ConfigChecker ====================
 */

// Utility tool to make checking configs easier and more readable.
class ConfigChecker {
 public:
  explicit ConfigChecker(std::string module_name)
      : name_(std::move(module_name)),
        print_width_(GlobalSettings().default_print_width) {}

  bool isValid(bool print_warnings = false) const {
    if (warnings_.empty()) {
      return true;
    }
    if (print_warnings) {
      std::string sev = "Warning: ";
      int length = print_width_ - sev.length();
      std::string warning =
          "\n" + internal::printCenter(name_, print_width_, '=');
      for (std::string w : warnings_) {
        std::string line = sev;
        while (w.length() > length) {
          line.append(w.substr(0, length));
          w = w.substr(length);
          warning.append("\n" + line);
          line = std::string(sev.length(), ' ');
        }
        warning.append("\n" + line + w);
      }
      warning = warning + "\n" + std::string(print_width_, '=');
      LOG(WARNING) << warning;
    }
    return false;
  }

  void checkValid() const { CHECK(isValid(true)); }

  void reset() { warnings_.clear(); }

  template <typename T>
  void checkGT(const T& param, const T& value, const std::string& name) {
    if (param <= value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected > '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkGE(const T& param, const T& value, const std::string& name) {
    if (param < value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected >= '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkLT(const T& param, const T& value, const std::string& name) {
    if (param >= value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected < '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkLE(const T& param, const T& value, const std::string& name) {
    if (param > value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected <= '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkEq(const T& param, const T& value, const std::string& name) {
    if (param != value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected to be '" << value << "' (is: '"
         << param << "').";
      warnings_.emplace_back(ss.str());
    }
  }

  template <typename T>
  void checkNE(const T& param, const T& value, const std::string& name) {
    if (param == value) {
      std::stringstream ss;
      ss << "Param '" << name << "' is expected to be different from '" << value
         << "'.";
      warnings_.emplace_back(ss.str());
    }
  }

  void checkCond(bool condition, const std::string& warning) {
    if (!condition) {
      warnings_.emplace_back(warning);
    }
  }

  void setPrintWidth(int width) { print_width_ = width; }

 private:
  const std::string name_;
  std::vector<std::string> warnings_;
  int print_width_;
};

/**
 * ==================== ConfigInternal ====================
 */
namespace internal {
// Base class for internal use.
struct ConfigInternal : public ConfigInternalVerificator {
 public:
  explicit ConfigInternal(std::string name)
      : name_(std::move(name)), meta_data_(new MetaData()) {}

  ConfigInternal(const ConfigInternal& other)
      : name_(other.name_), meta_data_(new MetaData(*(other.meta_data_))) {}

  ConfigInternal& operator=(const ConfigInternal& other) {
    name_ = other.name_;
    meta_data_.reset(new MetaData(*(other.meta_data_)));
    return *this;
  }

  bool isValid(bool print_warnings = false) const {
    meta_data_->checker = std::make_unique<ConfigChecker>(name_);
    meta_data_->checker->setPrintWidth(meta_data_->print_width);
    meta_data_->print_warnings = print_warnings;
    checkParams();
    bool result = meta_data_->checker->isValid(print_warnings);
    meta_data_->checker.reset(nullptr);
    return result;
  }

  std::string toString() const {
    meta_data_->messages = std::make_unique<std::vector<std::string>>();
    meta_data_->merged_setup_already_used = true;
    meta_data_->merged_setup_set_params = false;
    meta_data_->merged_setup_currently_active = true;
    // NOTE: setupParamsAndPrinting() does not modify 'this' in printing mode.
    ((ConfigInternal*)this)->setupParamsAndPrinting();
    printFields();
    std::string result =
        internal::printCenter(name_, meta_data_->print_width, '=');
    for (const std::string& msg : *(meta_data_->messages)) {
      result.append("\n" + msg);
    }
    result.append("\n" + std::string(meta_data_->print_width, '='));
    meta_data_->messages.reset(nullptr);
    return result;
  };

  // Implementable setup tool.
  virtual void initializeDependentVariableDefaults() {}

  virtual void checkParams() const {}

  virtual void printFields() const {
    if (!meta_data_->merged_setup_already_used) {
      meta_data_->messages->emplace_back(
          std::string(meta_data_->indent, ' ')
              .append("The 'printFields()' method is not implemented for " +
                      name_ + "'."));
    }
  }

  virtual void fromRosParam() {
    if (!meta_data_->merged_setup_already_used) {
      LOG(WARNING) << "fromRosParam() is not implemented for '" << name_
                   << "', no parameters will be loaded.";
    }
  }

  virtual void setupParamsAndPrinting() {
    // If this is overwritten this won't be set and will precede fromRosParam
    // and printField.
    meta_data_->merged_setup_already_used = false;
  }

  // General Tools.
  void setConfigName(const std::string& name) { name_ = name; }
  void setPrintWidth(int width) { meta_data_->print_width = width; }
  void setPrintIndent(int indent) { meta_data_->print_indent = indent; }

 protected:
  // Checking Tools.
  template <typename T>
  void checkParamGT(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamGT()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkGT(param, value, name);
  }

  template <typename T>
  void checkParamGE(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamGE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkGE(param, value, name);
  }

  template <typename T>
  void checkParamLT(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamLT()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkLT(param, value, name);
  }

  template <typename T>
  void checkParamLE(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamLE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkLE(param, value, name);
  }

  template <typename T>
  void checkParamEq(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamEq()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkEq(param, value, name);
  }

  template <typename T>
  void checkParamNE(const T& param, const T& value,
                    const std::string& name) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamNE()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkNE(param, value, name);
  }

  void checkParamCond(bool condition, const std::string& warning) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamCond()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    meta_data_->checker->checkCond(condition, warning);
  }
  void checkParamConfig(const internal::ConfigInternal& config) const {
    if (!meta_data_->checker) {
      LOG(WARNING) << "'checkParamConfig()' calls are only allowed within the "
                      "'checkParams()' method, no checks will be performed.";
      return;
    }
    if (!config.isValid(meta_data_->print_warnings)) {
      meta_data_->checker->checkCond(
          false, "Member config '" + config.name_ + "' is not valid.");
    }
  }

  // Printing Tools.
  template <typename T>
  void printField(const std::string& name, const T& field) const {
    if (!meta_data_->messages) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'printField()' calls are only allowed within the "
                        "'printFields()' method.";
      }
      return;
    }
    if (isConfig(&field)) {
      printConfigInternal(name, (const internal::ConfigInternal*)&field);
    } else {
      std::stringstream ss;
      ss << field;
      printFieldInternal(name, ss.str());
    }
  }

  void printField(const std::string& name, const bool& field) const {
    std::string val = "False";
    if (field) {
      val = "True";
    }
    printFieldInternal(name, val);
  }

  template <typename T>
  void printField(const std::string& name, const std::vector<T>& field) const {
    if (!meta_data_->messages) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'printField()' calls are only allowed within the "
                        "'printFields()' method.";
      }
      return;
    }
    std::stringstream ss;
    ss << "[";
    size_t offset = 0;
    for (const T& value : field) {
      ss << value << ", ";
      offset = 2;
    }
    std::string s = ss.str();
    s = s.substr(0, s.length() - offset).append("]");
    printFieldInternal(name, s);
  }

  template <typename T>
  void printField(const std::string& name,
                  const std::map<std::string, T>& field) const {
    if (!meta_data_->messages) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'printField()' calls are only allowed within the "
                        "'printFields()' method.";
      }
      return;
    }
    std::stringstream ss;
    ss << "{";
    size_t offset = 0;
    for (auto it = field.begin(); it != field.end(); ++it) {
      ss << it->first << ": " << it->second << ", ";
      offset = 2;
    }
    std::string s = ss.str();
    s = s.substr(0, s.length() - offset).append("}");
    printFieldInternal(name, s);
  }

  void printText(const std::string& text) const {
    if (!meta_data_->messages) {
      LOG(WARNING) << "'printText()' calls are only allowed within the "
                      "'printFields()' method.";
      return;
    }
    meta_data_->messages->emplace_back(
        std::string(meta_data_->indent, ' ').append(text));
  }

 private:
  void printFieldInternal(const std::string& name,
                          const std::string& field) const {
    std::string f = field;

    // The header is the field name.
    std::string header = std::string(meta_data_->indent, ' ') + name + ": ";
    while (header.length() > meta_data_->print_width) {
      // Linebreaks for too long lines.
      meta_data_->messages->emplace_back(
          header.substr(0, meta_data_->print_width));
      header = header.substr(meta_data_->print_width);
    }
    if (header.length() < meta_data_->print_indent) {
      header.append(
          std::string(meta_data_->print_indent - header.length(), ' '));
    } else if (meta_data_->print_width - header.length() < f.length()) {
      meta_data_->messages->emplace_back(header);
      header = std::string(meta_data_->print_indent, ' ');
    }

    // First line could be shorter.
    int length = meta_data_->print_width - header.length();
    if (f.length() > length) {
      meta_data_->messages->emplace_back(header + f.substr(0, length));
      f = f.substr(length);

      // Fill the rest.
      length = meta_data_->print_width - meta_data_->print_indent;
      while (f.length() > length) {
        meta_data_->messages->emplace_back(
            std::string(meta_data_->print_indent, ' ') + f.substr(0, length));
        f = f.substr(length);
      }
      meta_data_->messages->emplace_back(
          std::string(meta_data_->print_indent, ' ') + f.substr(0, length));
    } else {
      meta_data_->messages->emplace_back(header.append(f));
    }
  }

  void printConfigInternal(const std::string& name,
                           const internal::ConfigInternal* field) const {
    meta_data_->messages->emplace_back(std::string(meta_data_->indent, ' ') +
                                       name + ":");
    meta_data_->messages->emplace_back(field->toStringInternal(
        meta_data_->indent +
            GlobalSettings::instance().default_subconfig_indent,
        meta_data_->print_width, meta_data_->print_indent));
  }

  std::string toStringInternal(int indent, int print_width,
                               int print_indent) const {
    int print_width_prev = meta_data_->print_width;
    int print_indent_prev = meta_data_->print_indent;
    int indent_prev = meta_data_->indent;
    meta_data_->print_width = print_width;
    meta_data_->print_indent = print_indent;
    meta_data_->indent = indent;

    meta_data_->messages = std::make_unique<std::vector<std::string>>();
    meta_data_->merged_setup_already_used = true;
    meta_data_->merged_setup_set_params = false;
    // NOTE: setupParamsAndPrinting() does not modify 'this' in printing mode.
    ((ConfigInternal*)this)->setupParamsAndPrinting();
    printFields();
    std::string result;
    for (const std::string& msg : *(meta_data_->messages)) {
      result.append("\n" + msg);
    }
    if (!result.empty()) {
      result = result.substr(1);
    }
    meta_data_->messages.reset(nullptr);
    meta_data_->print_width = print_width_prev;
    meta_data_->print_indent = print_indent_prev;
    meta_data_->indent = indent_prev;
    return result;
  };

  void setupFromParamMap(const internal::ParamMap& params) {
    param_namespace_ = "/";
    if (params.find("_name_space") != params.end()) {
      xmlCast(params.at("_name_space"), &param_namespace_);
    }

    // Read params
    meta_data_->params = &params;
    meta_data_->merged_setup_already_used = true;
    meta_data_->merged_setup_set_params = true;
    meta_data_->merged_setup_currently_active = true;
    setupParamsAndPrinting();
    fromRosParam();
    meta_data_->params = nullptr;
    initializeDependentVariableDefaults();
  }

  template <typename T>
  void rosParamInternal(const std::string& name, T* param) {
    CHECK_NOTNULL(param);
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                        "'fromRosParam()' method, no param will be loaded.";
      }
      return;
    }
    // Check the param is set.
    auto it = meta_data_->params->find(param_namespace_ + "/" + name);
    if (it == meta_data_->params->end()) {
      return;
    }
    // Set the param.
    if (!internal::xmlCast(it->second, param)) {
      LOG(WARNING) << name_ << ": param '" << name
                   << "' is set but could not be read.";
    }
  }

  template <typename T>
  void rosParamMapInternal(const std::string& name,
                           std::map<std::string, T>* param) {
    CHECK_NOTNULL(param);
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                        "'fromRosParam()' method, no param will be loaded.";
      }
      return;
    }
    // Check the param is set.
    std::map<std::string, T> values;
    for (const auto& v : *(meta_data_->params)) {
      if (v.first.find(name + "/") != 0) {
        continue;
      }
      std::string key = v.first.substr(name.length() + 1);

      if (key.find('/') == std::string::npos) {
        T value;
        if (!internal::xmlCast(v.second, &value)) {
          LOG(WARNING) << name_ << ": param '" << name
                       << "' is set but could not be read.";
          return;
        }
        values[key] = value;
      }
    }
    *param = values;
  }

  template <typename T>
  void setupParamInternal(const std::string& name, T* param) {
    if (meta_data_->merged_setup_set_params) {
      rosParam(name, param);
    } else {
      printField(name, *param);
    }
  }

 protected:
  // These are explicitly overloaded for agreement with ROS-params and to
  // prevent messy compilation errors.
  void rosParam(const std::string& name, int* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, float* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, double* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, bool* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::string* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<int>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<double>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<float>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<bool>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::vector<std::string>* param) {
    this->rosParamInternal(name, param);
  }

  void rosParam(const std::string& name, std::map<std::string, int>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name, std::map<std::string, double>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name, std::map<std::string, float>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name, std::map<std::string, bool>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name,
                std::map<std::string, std::string>* param) {
    this->rosParamMapInternal(name, param);
  }

  void rosParam(const std::string& name, XmlRpc::XmlRpcValue* param) {
    CHECK_NOTNULL(param);
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                        "'fromRosParam()' method, no param will be loaded.";
      }
      return;
    }
    // Check the param is set.
    auto it = meta_data_->params->find(param_namespace_ + "/" + name);
    if (it == meta_data_->params->end()) {
      return;
    }
    *param = it->second;
  }

  void rosParam(ConfigInternal* config, const std::string& sub_namespace = "") {
    CHECK_NOTNULL(config);
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                        "'fromRosParam()' method, no param will be loaded.";
      }
      return;
    }

    // Update the sub-namespace. Default to same ns. Leading "/" for global ns.
    // Leading "~" for private ns.
    internal::ParamMap params = *(meta_data_->params);
    if (sub_namespace.empty()) {
      params["_name_space"] = param_namespace_;
    } else if (sub_namespace.front() == '/') {
      params["_name_space"] = sub_namespace;
    } else if (sub_namespace.front() == '~') {
      params["_name_space"] =
          static_cast<std::string>(params["_name_space_private"]) +
          sub_namespace.substr(1);
    } else {
      params["_name_space"] = param_namespace_ + "/" + sub_namespace;
    }
    setupConfigFromParamMap(params, config);
  }

  std::string rosParamNameSpace() {
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING)
            << "'rosParamNameSpace()' calls are only allowed within the "
               "'fromRosParam()' method, no param will be loaded.";
      }
    }
    return param_namespace_;
  }

  void setupParam(const std::string& name, int* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, float* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, double* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, bool* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, std::string* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, std::vector<int>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, std::vector<double>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, std::vector<float>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, std::vector<bool>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, std::vector<std::string>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, std::map<std::string, int>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name,
                  std::map<std::string, double>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name,
                  std::map<std::string, float>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, std::map<std::string, bool>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name,
                  std::map<std::string, std::string>* param) {
    this->setupParamInternal(name, param);
  }

  void setupParam(const std::string& name, ConfigInternal* config,
                  const std::string& sub_namespace = "") {
    if (meta_data_->merged_setup_set_params) {
      rosParam(config, sub_namespace);
    } else {
      printField(name, *config);
    }
  }

#ifdef CONFIG_UTILITIES_TRANSFORMATION_ENABLED
  template <typename Scalar>
  void rosParam(const std::string& name,
                kindr::minimal::QuatTransformationTemplate<Scalar>* param) {
    CHECK_NOTNULL(param);
    // Check scope and param map are valid.
    if (!meta_data_->params) {
      if (!meta_data_->merged_setup_currently_active) {
        LOG(WARNING) << "'rosParam()' calls are only allowed within the "
                        "'fromRosParam()' method, no param will be loaded.";
      }
      return;
    }
    // Check the param is set.
    auto it = meta_data_->params->find(param_namespace_ + "/" + name);
    if (it == meta_data_->params->end()) {
      return;
    }
    XmlRpc::XmlRpcValue xml = it->second;
    // NOTE: This code was taken and adapted from minkindr_conversions:
    // https://github.com/ethz-asl/minkindr_ros/blob/master/
    // minkindr_conversions/include/minkindr_conversions/kindr_xml.h
    if (xml.size() != 4) {
      LOG(WARNING) << name_ << ": param '" << name
                   << "' is set but could not be read.";
      return;
    }
    // read raw inputs
    typename kindr::minimal::QuatTransformationTemplate<Scalar>::RotationMatrix
        temp_rot_matrix;
    typename kindr::minimal::QuatTransformationTemplate<Scalar>::Position
        temp_translation;
    for (size_t i = 0; i < 3; ++i) {
      if (xml[i].size() != 4) {
        LOG(WARNING) << name_ << ": param '" << name
                     << "' is set but could not be read.";
        return;
      }
      for (size_t j = 0; j < 3; ++j) {
        temp_rot_matrix(i, j) = static_cast<double>(xml[i][j]);
      }
      temp_translation(i) = static_cast<double>(xml[i][3]);
    }

    // renormalize rotation to correct for rounding error when yaml was written
    kindr::minimal::RotationQuaternionTemplate<Scalar> temp_rot_quat =
        kindr::minimal::RotationQuaternionTemplate<
            Scalar>::constructAndRenormalize(temp_rot_matrix);

    // recombine
    *param = kindr::minimal::QuatTransformationTemplate<Scalar>(
        temp_rot_quat, temp_translation);
  }

  template <typename Scalar>
  void printField(
      const std::string& name,
      const kindr::minimal::QuatTransformationTemplate<Scalar>& field) const {
    if (!meta_data_->messages) {
      LOG(WARNING) << "'printField()' calls are only allowed within the "
                      "'printFields()' method.";
      return;
    }
    std::stringstream ss;
    auto rot =
        field.getEigenQuaternion().toRotationMatrix().eulerAngles(0, 1, 2);
    rot = rot * 180.0 / M_PI;
    ss << "t: [" << field.getPosition()[0] << ", " << field.getPosition()[1]
       << ", " << field.getPosition()[2] << "] RPY°: [" << rot.x() << ", "
       << rot.y() << ", " << rot.z() << "]";
    printFieldInternal(name, ss.str());
  }

  template <typename Scalar>
  void setupParam(const std::string& name,
                  kindr::minimal::QuatTransformationTemplate<Scalar>* param) {
    this->setupParamInternal(name, param);
  }
#endif  // CONFIG_UTILITIES_TRANSFORMATION_ENABLED

 private:
  friend void setupConfigFromParamMap(const internal::ParamMap& params,
                                      ConfigInternal* config);

  struct MetaData {
    // tools
    std::unique_ptr<ConfigChecker> checker;
    std::unique_ptr<std::vector<std::string>> messages;
    const internal::ParamMap* params = nullptr;

    // settings and variables
    int print_width = GlobalSettings::instance().default_print_width;
    int print_indent = GlobalSettings::instance().default_print_indent;
    int indent = 0;  // Only used for nested printing.
    bool print_warnings = false;
    bool merged_setup_already_used = false;
    bool merged_setup_set_params = false;
    bool merged_setup_currently_active = false;

    MetaData() = default;
    MetaData(const MetaData& other) {
      print_width = other.print_width;
      print_indent = other.print_indent;
      indent = other.indent;
    }
  };

  std::string name_;
  std::string param_namespace_ = "~";
  std::unique_ptr<MetaData> meta_data_;
};

// This is a dummy operator, configs provide toString().
inline std::ostream& operator<<(std::ostream& os, const ConfigInternal&) {
  return os;
}

/**
 * ==================== Exposure Utilities ===================
 */

inline void setupConfigFromParamMap(const ParamMap& params,
                                    ConfigInternal* config) {
  CHECK_NOTNULL(config);
  config->setupFromParamMap(params);
}

}  // namespace internal

/**
 * ==================== Config ====================
 */
template <typename ConfigT>
struct Config : public internal::ConfigInternal {
 public:
  // Construction.
  Config() : ConfigInternal(typeid(ConfigT).name()) {}

  ConfigT checkValid() const {
    // Returns a copy of the config in the const case.
    CHECK(isValid(true));
    ConfigT result(*static_cast<const ConfigT*>(this));
    return result;
  }

  ConfigT& checkValid() {
    // Returns a mutable reference.
    CHECK(isValid(true));
    return *static_cast<ConfigT*>(this);
  }
};

/**
 * ==================== Factory ====================
 */
class Factory {
 public:
  // Registration.
  template <class BaseT, class DerivedT, typename... Args>
  struct Registration {
    explicit Registration(const std::string& type) {
      ModuleMap<BaseT, Args...>::instance().template addEntry<DerivedT>(type);
    }
  };

  template <class BaseT, class DerivedT, typename... Args>
  struct RegistrationRos {
    explicit RegistrationRos(const std::string& type) {
      ModuleMap<BaseT, Args...>::instance().template addEntryRos<DerivedT>(
          type);
    }
  };

  // Creation.
  template <class BaseT, typename... Args>
  static std::unique_ptr<BaseT> create(const std::string& type, Args... args) {
    ModuleMap<BaseT, Args...>& module = ModuleMap<BaseT, Args...>::instance();
    std::stringstream ss;
    ((ss << typeid(args).name() << ", "), ...);
    std::string type_info = ss.str();
    if (!type_info.empty()) {
      type_info = " and constructor arguments '" +
                  type_info.substr(0, type_info.size() - 2) + "'";
    } else {
      type_info = "";
    }
    if (module.map.empty()) {
      LOG(ERROR) << "Cannot create a module of type '" << type
                 << "': No modules registered to the factory for base '"
                 << typeid(BaseT).name() << "'" << type_info
                 << ". Register modules using a static Registration struct.";
      return nullptr;
    }
    auto it = module.map.find(type);
    if (it == module.map.end()) {
      std::string module_list;
      for (const auto& entry : module.map) {
        module_list.append(entry.first + ", ");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      LOG(ERROR) << "No module of type '" << type
                 << "' registered to the factory for base '"
                 << typeid(BaseT).name() << "'" << type_info
                 << ". Registered are: " << module_list << ".";
      return nullptr;
    }
    return std::unique_ptr<BaseT>(it->second(args...));
  }

 protected:
  template <class BaseT, typename... Args>
  struct ModuleMap {
   public:
    using FactoryMethod = std::function<BaseT*(Args... args)>;
    using FactoryMethodRos =
        std::function<BaseT*(const internal::ParamMap& params, Args... args)>;

    // Singleton access.
    static ModuleMap& instance() {
      static ModuleMap instance_;
      return instance_;
    }

    // Add entries.
    template <class DerivedT>
    void addEntry(const std::string& type) {
      if (map.find(type) != map.end()) {
        LOG(ERROR) << "Cannot register already existent type '" << type
                   << "' for <DerivedT>='" << typeid(DerivedT).name()
                   << "' to factory for base '" << typeid(BaseT).name() << "'.";
      } else {
        map.insert(std::make_pair(
            type, [](Args... args) { return new DerivedT(args...); }));
      }
    }

    template <class DerivedT>
    void addEntryRos(const std::string& type) {
      if (map_ros.find(type) != map_ros.end()) {
        LOG(ERROR) << "Cannot register already existent type '" << type
                   << "' for <DerivedT>='" << typeid(DerivedT).name()
                   << "' to factory for base '" << typeid(BaseT).name() << "'.";
      } else {
        map_ros.insert(std::make_pair(
            type,
            [type](const internal::ParamMap& params, Args... args) -> BaseT* {
              typename DerivedT::Config config;
              auto config_ptr =
                  dynamic_cast<Config<typename DerivedT::Config>*>(&config);
              if (!config_ptr) {
                LOG(ERROR) << "Cannot create '" << type << "' with <DerivedT>='"
                           << typeid(DerivedT).name()
                           << "': 'DerivedT::Config' needs to inherit from "
                              "'config_utilities::Config<DerivedT::Config>'.";
                return nullptr;
              }
              internal::setupConfigFromParamMap(params, config_ptr);
              return new DerivedT(config, args...);
            }));
      }
    }

    // The maps.
    std::unordered_map<std::string, FactoryMethod> map;
    std::unordered_map<std::string, FactoryMethodRos> map_ros;

   private:
    ModuleMap() = default;
  };
};

}  // namespace config_utilities
#endif  // CONFIG_UTILITIES_CORE_HPP_

/**
 * ==================== ROS Tools ====================
 */
#ifdef CONFIG_UTILITIES_ROS_ENABLED
#ifndef CONFIG_UTILITIES_ROS_HPP_
#define CONFIG_UTILITIES_ROS_HPP_
namespace config_utilities {

namespace internal {
inline ParamMap getParamMapFromRos(const ros::NodeHandle& nh) {
  internal::ParamMap params;
  std::vector<std::string> keys;
  XmlRpc::XmlRpcValue value;
  const std::string& ns = nh.getNamespace();
  nh.getParamNames(keys);
  for (std::string& key : keys) {
    nh.getParam(key, value);
    params[key] = value;
  }
  params["_name_space"] = ns;
  params["_name_space_private"] = ns;
  return params;
}
}  // namespace internal

// Tool to create configs from ROS.
template <typename ConfigT>
ConfigT getConfigFromRos(const ros::NodeHandle& nh) {
  ConfigT config;
  if (!internal::isConfig(&config)) {
    LOG(ERROR) << "Cannot 'getConfigFromRos()' for <ConfigT>='"
               << typeid(ConfigT).name()
               << "' that does not inherit from "
                  "'config_utilities::Config<ConfigT>'.";
    return config;
  }
  auto config_ptr = dynamic_cast<Config<ConfigT>*>(&config);
  if (!config_ptr) {
    LOG(ERROR) << "Cannot 'getConfigFromRos()' for <ConfigT>='"
               << typeid(ConfigT).name()
               << "' that does not inherit from "
                  "'config_utilities::Config<ConfigT>'.";
    return config;
  }

  // Setup.
  internal::ParamMap params = internal::getParamMapFromRos(nh);
  internal::setupConfigFromParamMap(params, config_ptr);
  return config;
}
/**
 * ==================== ROS Factory ====================
 */

class FactoryRos : protected Factory {
 public:
  // Creation.
  template <class BaseT, typename... Args>
  static std::unique_ptr<BaseT> create(const ros::NodeHandle& nh,
                                       Args... args) {
    ModuleMap<BaseT, Args...>& module = ModuleMap<BaseT, Args...>::instance();
    std::stringstream ss;
    ((ss << typeid(args).name() << ", "), ...);
    std::string type_info = ss.str();
    if (!type_info.empty()) {
      type_info = " and constructor arguments '" +
                  type_info.substr(0, type_info.size() - 2) + "'";
    } else {
      type_info = "";
    }

    // Get the type from param.
    std::string type;
    if (!nh.hasParam("type")) {
      LOG(ERROR) << "ROS factory creation requires the param 'type' to be set "
                    "in namespace '"
                 << nh.getNamespace() << "'.";
      return nullptr;
    }
    nh.getParam("type", type);

    if (module.map_ros.empty()) {
      LOG(ERROR) << "Cannot create a module of type '" << type
                 << "': No modules registered to the factory for base '"
                 << typeid(BaseT).name() << "'" << type_info
                 << ". Register modules using a static Registration struct.";
      return nullptr;
    }
    auto it = module.map_ros.find(type);
    if (it == module.map_ros.end()) {
      std::string module_list;
      for (const auto& entry : module.map_ros) {
        module_list.append(entry.first + ", ");
      }
      module_list = module_list.substr(0, module_list.size() - 2);
      LOG(ERROR) << "No module of type '" << type
                 << "' registered to the factory for base '"
                 << typeid(BaseT).name() << "'" << type_info
                 << ". Registered are: " << module_list << ".";
      return nullptr;
    }

    // Get the config and create the target.
    internal::ParamMap params = internal::getParamMapFromRos(nh);
    return std::unique_ptr<BaseT>(it->second(params, args...));
  }
};

}  // namespace config_utilities
#endif  // CONFIG_UTILITIES_ROS_HPP_
#endif  // CONFIG_UTILITIES_ROS_ENABLED
