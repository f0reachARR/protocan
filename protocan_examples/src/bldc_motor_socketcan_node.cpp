#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "bldc_motor.hpp"
#include "protocan/types.hpp"
#include "protocan_device/device.hpp"
#include "socketcan_device_interface.hpp"

namespace
{

std::atomic<bool> g_running{true};

uint32_t monotonic_ms()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

uint32_t rand_ms(uint32_t min_ms, uint32_t max_ms)
{
  static uint32_t seed = 0x1234ABCDu;
  seed = seed * 1664525u + 1013904223u;
  const uint32_t span = (max_ms >= min_ms) ? (max_ms - min_ms + 1u) : 1u;
  return min_ms + (seed % span);
}

void on_signal(int)
{
  g_running.store(false);
}

struct AppState
{
  bool enabled = false;
  float target_linear_x = 0.0F;
  float target_angular_z = 0.0F;
  float velocity_rps = 0.0F;
  float temperature_c = 27.0F;
  float pid_kp = 0.20F;
  float pid_ki = 0.01F;
  float pid_kd = 0.00F;
};

void on_cmd_vel(const protocan::bldc_motor::TwistCommand & msg, void * ctx)
{
  auto * state = static_cast<AppState *>(ctx);
  state->target_linear_x = msg.linear_x;
  state->target_angular_z = msg.angular_z;
}

protocan::Status on_set_enable(
  const protocan::bldc_motor::SetEnableRequest & req,
  protocan::bldc_motor::SetEnableResponse & res,
  void * ctx)
{
  auto * state = static_cast<AppState *>(ctx);
  state->enabled = req.enable;
  res.success = true;
  return protocan::Status::OK;
}

void write_f32_le(uint8_t * out, float value)
{
  uint32_t u = 0;
  std::memcpy(&u, &value, sizeof(float));
  out[0] = static_cast<uint8_t>(u);
  out[1] = static_cast<uint8_t>(u >> 8);
  out[2] = static_cast<uint8_t>(u >> 16);
  out[3] = static_cast<uint8_t>(u >> 24);
}

float read_f32_le(const uint8_t * in)
{
  const uint32_t u =
    static_cast<uint32_t>(in[0]) |
    (static_cast<uint32_t>(in[1]) << 8) |
    (static_cast<uint32_t>(in[2]) << 16) |
    (static_cast<uint32_t>(in[3]) << 24);
  float out = 0.0F;
  std::memcpy(&out, &u, sizeof(float));
  return out;
}

protocan::Status on_get_pid_gains(uint8_t * out, uint8_t & size, void * ctx)
{
  auto * state = static_cast<AppState *>(ctx);
  write_f32_le(out + 0, state->pid_kp);
  write_f32_le(out + 4, state->pid_ki);
  write_f32_le(out + 8, state->pid_kd);
  size = 12;
  return protocan::Status::OK;
}

protocan::Status on_set_pid_gains(const uint8_t * data, uint8_t size, void * ctx)
{
  if (size < 12) {
    return protocan::Status::INVALID_ARGUMENT;
  }

  auto * state = static_cast<AppState *>(ctx);
  state->pid_kp = read_f32_le(data + 0);
  state->pid_ki = read_f32_le(data + 4);
  state->pid_kd = read_f32_le(data + 8);
  return protocan::Status::OK;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);

  const std::string iface = (argc >= 2) ? argv[1] : "vcan0";
  const uint8_t device_id = (argc >= 3) ? static_cast<uint8_t>(std::stoi(argv[2])) : 1;

  SocketCanDeviceInterface can_if(iface);
  if (can_if.open() != protocan::Status::OK) {
    std::cerr << "Failed to open SocketCAN interface: " << iface << '\n';
    return 1;
  }

  AppState state;

  protocan::bldc_motor::Node node(/*local_id=*/1, "bldc_motor_example");
  node.on_cmd_vel(&on_cmd_vel, &state);
  node.on_set_enable(&on_set_enable, &state);
  node.on_get_pid_gains(&on_get_pid_gains, &state);
  node.on_set_pid_gains(&on_set_pid_gains, &state);

  protocan::device::Device device(device_id, &can_if, &monotonic_ms, &rand_ms);
  if (device.add_node(node) != protocan::Status::OK) {
    std::cerr << "Failed to add node to device instance\n";
    return 2;
  }
  device.start();

  uint32_t last_status_publish_ms = 0;
  uint32_t last_log_ms = 0;

  while (g_running.load()) {
    device.poll();

    const uint32_t now_ms = monotonic_ms();
    if (now_ms - last_status_publish_ms >= 100) {
      if (state.enabled) {
        state.velocity_rps += (state.target_linear_x - state.velocity_rps) * 0.2F;
      } else {
        state.velocity_rps *= 0.85F;
      }
      state.temperature_c += 0.01F + std::fabs(state.velocity_rps) * 0.001F;

      auto & status = node.status_buffer();
      status.current_a = state.enabled ? std::fabs(state.target_linear_x) * 1.5F : 0.0F;
      status.velocity_rps = state.velocity_rps;
      status.temperature_c = state.temperature_c;
      status.error_flags = 0;

      (void)node.publish_status();
      last_status_publish_ms = now_ms;
    }

    if (now_ms - last_log_ms >= 1000) {
      std::cout
        << "iface=" << iface
        << " dev=" << static_cast<int>(device_id)
        << " enabled=" << (state.enabled ? "1" : "0")
        << " vel=" << state.velocity_rps
        << " temp=" << state.temperature_c
        << '\n';
      last_log_ms = now_ms;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  (void)can_if.close();
  return 0;
}
