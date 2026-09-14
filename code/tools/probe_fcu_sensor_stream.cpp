// Standalone TCP probe. Only GCS heartbeat and optional sensor stream requests.
#include <arpa/inet.h>
#include <sys/socket.h>
#include <poll.h>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include "common/mavlink.h"

int main(int argc, char **argv) {
  if (argc != 4 || (std::string(argv[3]) != "observe" &&
                    std::string(argv[3]) != "request")) {
    std::cerr << "Usage: probe_fcu_sensor_stream IPV4 PORT observe|request\n";
    return 2;
  }
  int fd = -1;
  try {
    sockaddr_in address{};
    address.sin_family = AF_INET;
    const int port = std::stoi(argv[2]);
    if (port < 1 || port > 65535 || inet_pton(AF_INET, argv[1], &address.sin_addr) != 1)
      throw std::runtime_error("Invalid endpoint");
    address.sin_port = htons(port);
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0 || connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)))
      throw std::runtime_error("TCP connect failed");
    auto transmit = [&](mavlink_message_t &message) {
      uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
      const auto length = mavlink_msg_to_send_buffer(buffer, &message);
      size_t sent = 0;
      while (sent < length) {
        const auto n = send(fd, buffer + sent, length - sent, MSG_NOSIGNAL);
        if (n <= 0) throw std::runtime_error("TCP send failed");
        sent += n;
      }
    };
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    double next_heartbeat = 0;
    uint8_t target_system = 0, target_component = 0;
    bool interval_sent = false, fallback_sent = false;
    unsigned received_count = 0;
    const bool request = std::string(argv[3]) == "request";
    mavlink_message_t received{};
    mavlink_status_t status{};
    while (true) {
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (elapsed >= (request ? 40 : 15)) break;
      mavlink_message_t outgoing{};
      if (elapsed >= next_heartbeat) {
        mavlink_msg_heartbeat_pack(255, 190, &outgoing, MAV_TYPE_GCS,
            MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
        transmit(outgoing);
        next_heartbeat = elapsed + 1;
      }
      if (request && target_system && elapsed >= 10 && !interval_sent) {
        for (int id : {26, 27, 105}) {
          mavlink_msg_command_long_pack(255, 190, &outgoing, target_system,
              target_component, MAV_CMD_SET_MESSAGE_INTERVAL, 0,
              id, 20000, 0, 0, 0, 0, 0);
          transmit(outgoing);
          std::cout << "{\"event\":\"request_interval\",\"elapsed_s\":" << elapsed
                    << ",\"message_id\":" << id << ",\"interval_us\":20000}\n";
        }
        interval_sent = true;
      }
      if (request && target_system && elapsed >= 25 && !fallback_sent) {
        mavlink_msg_request_data_stream_pack(255, 190, &outgoing, target_system,
            target_component, MAV_DATA_STREAM_RAW_SENSORS, 50, 1);
        transmit(outgoing);
        std::cout << "{\"event\":\"request_raw_sensors\",\"elapsed_s\":" << elapsed << "}\n";
        fallback_sent = true;
      }
      pollfd descriptor{fd, POLLIN, 0};
      if (poll(&descriptor, 1, 100) < 0) throw std::runtime_error("poll failed");
      if (!(descriptor.revents & (POLLIN | POLLHUP | POLLERR))) continue;
      uint8_t buffer[4096];
      const auto n = recv(fd, buffer, sizeof(buffer), 0);
      if (n <= 0) throw std::runtime_error("TCP connection closed");
      for (ssize_t i = 0; i < n; ++i) {
        if (!mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &received, &status)) continue;
        ++received_count;
        std::cout << "{\"elapsed_s\":" << elapsed << ",\"message_id\":" << unsigned(received.msgid)
                  << ",\"system\":" << int(received.sysid)
                  << ",\"component\":" << int(received.compid);
        if (received.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          mavlink_heartbeat_t value{};
          mavlink_msg_heartbeat_decode(&received, &value);
          if (value.type != MAV_TYPE_GCS && value.autopilot != MAV_AUTOPILOT_INVALID) {
            target_system = received.sysid;
            target_component = received.compid;
          }
          std::cout << ",\"base_mode\":" << int(value.base_mode)
                    << ",\"system_status\":" << int(value.system_status);
        } else if (received.msgid == MAVLINK_MSG_ID_BATTERY_STATUS) {
          mavlink_battery_status_t value{};
          mavlink_msg_battery_status_decode(&received, &value);
          std::cout << ",\"uwb_raw_cm\":[";
          for (int slot = 2; slot < 6; ++slot)
            std::cout << (slot == 2 ? "" : ",") << value.voltages[slot];
          std::cout << "]";
        } else if (received.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
          mavlink_command_ack_t value{};
          mavlink_msg_command_ack_decode(&received, &value);
          std::cout << ",\"command\":" << value.command << ",\"result\":" << int(value.result);
        }
        // Preserve payload bytes for later decoding without assuming vendor units.
        std::cout << ",\"payload_hex\":\"";
        const auto *payload = reinterpret_cast<const uint8_t *>(_MAV_PAYLOAD(&received));
        const char *hex = "0123456789abcdef";
        for (int j = 0; j < received.len; ++j)
          std::cout << hex[payload[j] >> 4] << hex[payload[j] & 15];
        std::cout << "\"}\n";
      }
    }
    close(fd);
    fd = -1;
    if (!received_count) throw std::runtime_error("No valid MAVLink messages received");
    if (request && !fallback_sent)
      throw std::runtime_error("No eligible flight-controller heartbeat; requests not sent");
    return 0;
  } catch (const std::exception &error) {
    if (fd >= 0) close(fd);
    std::cerr << error.what() << '\n';
    return 1;
  }
}
