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

  void setup() override;
  void loop() override;
  void receive_frame();
  void set_frequency_sweep(uint32_t start_hz, uint32_t end_hz, uint32_t step_hz,
                           uint32_t interval_ms);

  void add_frame_handler(std::function<void(Frame *)> &&callback);

protected:
  static void wakeup_receiver_task_from_isr(TaskHandle_t *arg);
  static void receiver_task(Radio *arg);
  bool apply_pending_frequency_change();

  RadioTransceiver *radio{nullptr};
  TaskHandle_t receiver_task_handle_{nullptr};
  QueueHandle_t packet_queue_{nullptr};
  bool frequency_sweep_enabled_{false};
  uint32_t sweep_start_hz_{868950000};
  uint32_t sweep_end_hz_{868950000};
  uint32_t sweep_step_hz_{0};
  uint32_t sweep_interval_ms_{0};
  uint32_t sweep_current_hz_{868950000};
  uint32_t last_sweep_ms_{0};
  volatile uint32_t pending_frequency_hz_{0};

  std::vector<std::function<void(Frame *)>> handlers_;
};
} // namespace wmbus_radio
} // namespace esphome
