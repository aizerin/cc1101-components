#pragma once

#include <functional>

#include "freertos/FreeRTOS.h"

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include "esphome/components/spi/spi.h"
#include "esphome/components/wmbus_common/wmbus.h"

#include "packet.h"
#include "transceiver.h"

namespace esphome {
namespace wmbus_radio {

class Radio : public Component {
public:
  void set_radio(RadioTransceiver *radio) { this->radio = radio; };
  void set_raw_rx(bool raw_rx) { this->raw_rx_ = raw_rx; };
  void set_rssi_threshold(int8_t rssi_threshold) {
    this->rssi_threshold_ = rssi_threshold;
  };

  void setup() override;
  void loop() override;
  void receive_frame();

  void add_frame_handler(std::function<void(Frame *)> &&callback);

protected:
  static void wakeup_receiver_task_from_isr(TaskHandle_t *arg);
  static void receiver_task(Radio *arg);

  RadioTransceiver *radio{nullptr};
  TaskHandle_t receiver_task_handle_{nullptr};
  QueueHandle_t packet_queue_{nullptr};

  // Raw RX diagnostic sniffer: dump bytes straight from radio, skip decoding.
  bool raw_rx_{false};
  // Drop captures weaker than this RSSI (dBm). -128 = disabled.
  int8_t rssi_threshold_{-128};
  // Diagnostics: count captured packets to reveal noise flooding (logged at WARN).
  uint32_t capture_count_{0};
  uint32_t last_rate_log_ms_{0};

  std::vector<std::function<void(Frame *)>> handlers_;
};
} // namespace wmbus_radio
} // namespace esphome
