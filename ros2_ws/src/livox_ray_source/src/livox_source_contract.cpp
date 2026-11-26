#include <utility>

#include "livox_ray_source/livox_sdk_sample_source.hpp"

namespace livox_ray_source
{

LivoxDeviceRegistry::LivoxDeviceRegistry(std::string primary_serial)
: _primary_serial(std::move(primary_serial))
{
}

void LivoxDeviceRegistry::clear()
{
  const std::lock_guard<std::mutex> lock(_mutex);
  _devices.clear();
  _primary_handle.reset();
  _multiple_device_fault = false;
}

void LivoxDeviceRegistry::observe(const LivoxDeviceIdentity & device)
{
  const std::lock_guard<std::mutex> lock(_mutex);
  _devices[device.handle] = device;

  if (!_primary_serial.empty()) {
    if (device.serial == _primary_serial) {
      _primary_handle = device.handle;
    }
    return;
  }

  if (!_primary_handle.has_value()) {
    _primary_handle = device.handle;
  }
  else if (*_primary_handle != device.handle) {
    _multiple_device_fault = true;
  }
}

void LivoxDeviceRegistry::setPrimarySerial(std::string serial)
{
  const std::lock_guard<std::mutex> lock(_mutex);
  _primary_serial = std::move(serial);
  _primary_handle.reset();
  _multiple_device_fault = false;
  for (const auto & [handle, device] : _devices) {
    if (!_primary_serial.empty()) {
      if (device.serial == _primary_serial) {
        _primary_handle = handle;
      }
    }
    else if (!_primary_handle.has_value()) {
      _primary_handle = handle;
    }
    else if (*_primary_handle != handle) {
      _multiple_device_fault = true;
    }
  }
}

bool LivoxDeviceRegistry::accepts(uint32_t handle) const
{
  const std::lock_guard<std::mutex> lock(_mutex);
  return !_multiple_device_fault && _primary_handle.has_value() && *_primary_handle == handle;
}

bool LivoxDeviceRegistry::multipleDeviceFault() const
{
  const std::lock_guard<std::mutex> lock(_mutex);
  return _multiple_device_fault;
}

std::optional<LivoxDeviceIdentity> LivoxDeviceRegistry::primaryDevice() const
{
  const std::lock_guard<std::mutex> lock(_mutex);
  if (!_primary_handle.has_value()) {
    return std::nullopt;
  }
  const auto it = _devices.find(*_primary_handle);
  return it == _devices.end() ? std::nullopt : std::optional<LivoxDeviceIdentity>(it->second);
}

std::vector<LivoxDeviceIdentity> LivoxDeviceRegistry::devices() const
{
  const std::lock_guard<std::mutex> lock(_mutex);
  std::vector<LivoxDeviceIdentity> result;
  result.reserve(_devices.size());
  for (const auto & [handle, device] : _devices) {
    static_cast<void>(handle);
    result.push_back(device);
  }
  return result;
}

}  // namespace livox_ray_source
