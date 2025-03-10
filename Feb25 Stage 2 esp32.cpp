// Modify the OnDataRecv callback function to implement handshake ack
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  struct_message *msg = (struct_message *)data;
  Serial.print("ESP-NOW message received: ");
  Serial.println(msg->command);

  if (strcmp(msg->command, "startCycle") == 0) {
    // Send handshake acknowledgment back to the sender
    struct_message ackMsg;
    memset(&ackMsg, 0, sizeof(ackMsg));
    strncpy(ackMsg.command, "ackStartCycle", sizeof(ackMsg.command) - 1);
    esp_err_t result = esp_now_send(mac_addr, (uint8_t *)&ackMsg, sizeof(ackMsg));
    if (result == ESP_OK) {
      Serial.println("Handshake ack sent to Stage 1");
    } else {
      Serial.println("Failed to send handshake ack to Stage 1");
    }

    // If the system is ready, start the cycle
    if (currentState == SystemState::READY) {
      Serial.println("🚀 Starting cycle from ESP-NOW message...");
      currentState = SystemState::CYCLE_RUNNING;
      runCuttingCycle();
      currentState = SystemState::READY;
    }
  }
}