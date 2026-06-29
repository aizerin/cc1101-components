#include "component.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "freertos/queue.h"
#include "freertos/task.h"

#define ASSERT(expr, expected, before_exit)                                    \
  {                                                                            \
    auto result = (expr);                                                      \
    if (!!result != expected) {                                                \
      ESP_LOGE(TAG, "Assertion failed: %s -> %d", #expr, result);              \
      before_exit;                                                             \
      return;                                                                  \
    }                                                                          \
  }

#define ASSERT_SETUP(expr) ASSERT(expr, 1, this->mark_failed())

namespace esphome {
namespace wmbus_radio {
static const char *TAG = "wmbus";

void Radio::setup() {
  ASSERT_SETUP(this->packet_queue_ = xQueueCreate(3, sizeof(Packet *)));

  // High priority to avoid FIFO overflow (fills in 5.12ms at 100kbps).
  // Pin to core 1 on dual-core to avoid WiFi ISR preemption on core 0.
#if portNUM_PROCESSORS > 1
  ASSERT_SETUP(xTaskCreatePinnedToCore((TaskFunction_t)this->receiver_task, "radio_recv",
                           8 * 1024, this, 24, &(this->receiver_task_handle_), 1));
#else
  ASSERT_SETUP(xTaskCreate((TaskFunction_t)this->receiver_task, "radio_recv",
                           8 * 1024, this, 24, &(this->receiver_task_handle_)));
#endif

  ESP_LOGI(TAG, "Receiver task created [%p]", this->receiver_task_handle_);

  this->radio->attach_data_interrupt(Radio::wakeup_receiver_task_from_isr,
                                     &(this->receiver_task_handle_));
}

void Radio::loop() {
  Packet *p;
  if (xQueueReceive(this->packet_queue_, &p, 0) != pdPASS)
    return;

  // ESP_LOGI(TAG, "Have RAW data from radio (%zu bytes)",
  //          p->calculate_payload_size());

  auto frame = p->convert_to_frame();

  if (!frame)
    return;

  ESP_LOGV(TAG, "Have data (%zu bytes) [RSSI: %ddBm, mode: %s %s]",
           frame->data().size(), frame->rssi(), toString(frame->link_mode()),
           frame->format().c_str());

  uint8_t packet_handled = 0;
  for (auto &handler : this->handlers_)
    handler(&frame.value());

  if (frame->handlers_count())
    ESP_LOGV(TAG, "Telegram handled by %d handlers", frame->handlers_count());
  else {
    ESP_LOGW(TAG, "Telegram not handled by any handler");
    Telegram t;
    if (t.parseHeader(frame->data()) && t.addresses.empty()) {
      ESP_LOGW(TAG, "Check if telegram can be parsed on:");
    } else {
      ESP_LOGW(TAG, "Check if telegram with address %s can be parsed on:",
               t.addresses.back().id.c_str());
    }
    ESP_LOGW(TAG,
             (std::string{"https://wmbusmeters.org/analyze/"} + frame->as_hex())
                 .c_str());
  }
}

void Radio::wakeup_receiver_task_from_isr(TaskHandle_t *arg) {
  BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(*arg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Radio::receive_frame() {
  // --- RSSI BAND SCANNER: pure energy detector, no sync/decode. Polls the
  // instantaneous RSSI fast and reports peak/floor once per second. A strong
  // peak when held near the meter proves antenna + reception work. ---
  if (this->radio->is_scan_mode()) {
    int8_t rssi = this->radio->get_rssi_inst();
    if (rssi > this->scan_peak_)
      this->scan_peak_ = rssi;
    if (rssi < this->scan_floor_)
      this->scan_floor_ = rssi;

    uint32_t now = millis();
    if (now - this->scan_log_ms_ >= 1000) {
      // Only log when the strongest sample in the window reaches the threshold,
      // so the noise floor doesn't spam the log. rssi_threshold default -128 =
      // log everything; set e.g. -80 to only see real signals.
      if (this->scan_peak_ >= this->rssi_threshold_)
        ESP_LOGW(TAG, "RSSI scan: floor=%d dBm  peak=%d dBm", this->scan_floor_,
                 this->scan_peak_);
      this->scan_log_ms_ = now;
      this->scan_peak_ = -128;
      this->scan_floor_ = 0;
    }
    delay(2);  // ~500 samples/s, catches short bursts
    return;
  }

  if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000))) {
    this->radio->restart_rx();
    return;
  }

  // --- DIAGNOSTICS: capture-rate counter (WARN so it shows without VERBOSE) ---
  // A high rate here means the radio is constantly tripping on noise, which
  // keeps it busy/blind and can starve real frames.
  this->capture_count_++;
  uint32_t now = millis();
  if (now - this->last_rate_log_ms_ >= 5000) {
    ESP_LOGW(TAG, "DIAG: %u RX captures in last %u ms", this->capture_count_,
             now - this->last_rate_log_ms_);
    this->last_rate_log_ms_ = now;
    this->capture_count_ = 0;
  }

  // --- RAW RX sniffer: dump a fixed window straight from the radio and bail out
  // before any wM-Bus decoding. Lets us see what actually arrives + RSSI. ---
  if (this->raw_rx_) {
    uint8_t buf[32] = {0};
    bool ok = this->radio->read_in_task(buf, sizeof(buf), 0);
    int8_t rssi = this->radio->get_rssi();
    ESP_LOGW(TAG, "RAW RX [rssi=%d dBm, read_ok=%d]: %s", rssi, ok,
             format_hex(buf, sizeof(buf)).c_str());
    this->radio->restart_rx();
    return;
  }

  auto packet = std::make_unique<Packet>();

  if (!this->radio->read_in_task(packet->rx_data_ptr(), packet->rx_capacity(), 0)) {
    this->radio->restart_rx();
    return;
  }

  if (!packet->calculate_payload_size()) {
    this->radio->restart_rx();
    return;
  }

  if (!this->radio->read_in_task(packet->rx_data_ptr(), packet->rx_capacity(), 3)) {
    this->radio->restart_rx();
    return;
  }

  int8_t rssi = this->radio->get_rssi();
  packet->set_rssi(rssi);

  // RSSI gate: drop weak captures (noise) so only strong signals proceed to
  // decoding/queue. Gives priority to nearby/real meter frames.
  if (rssi < this->rssi_threshold_) {
    ESP_LOGW(TAG, "DIAG: dropped weak capture [RSSI: %d dBm < threshold %d]", rssi,
             this->rssi_threshold_);
    this->radio->restart_rx();
    return;
  }

  // DIAG: log RSSI of every captured packet (even ones that fail CRC later),
  // so real signal (strong RSSI) can be told apart from noise (noise floor).
  ESP_LOGW(TAG, "DIAG: captured packet [RSSI: %d dBm]", rssi);

  // Re-arm sync word detector for next packet
  this->radio->restart_rx();

  auto packet_ptr = packet.get();

  if (xQueueSend(this->packet_queue_, &packet_ptr, 0) == pdTRUE) {
    ESP_LOGV(TAG, "Queue items: %zu",
             uxQueueMessagesWaiting(this->packet_queue_));
    ESP_LOGV(TAG, "Queue send success");
    packet.release();
  } else
    ESP_LOGW(TAG, "Queue send failed");
}

void Radio::receiver_task(Radio *arg) {
  while (true)
    arg->receive_frame();
}

void Radio::add_frame_handler(std::function<void(Frame *)> &&callback) {
  this->handlers_.push_back(std::move(callback));
}

} // namespace wmbus_radio
} // namespace esphome
