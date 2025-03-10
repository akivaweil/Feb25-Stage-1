// Add these declarations at the top (after existing includes and global variables)
volatile bool waitingForAck = false;
unsigned long handshakeTimestamp = 0;
const unsigned long HANDSHAKE_TIMEOUT = 3000; // 3 seconds timeout for handshake

// Add the following callback function:
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  struct_message *msg = (struct_message *)data;
  Serial.print("ESP-NOW message received: ");
  Serial.println(msg->command);
  if (strcmp(msg->command, "ackStartCycle") == 0) {
    waitingForAck = false;
    Serial.println("Received handshake ack from Stage 2");
  }
}

// In initESPNOW(), add registration for the receive callback:
// ... existing code ...
// After calling esp_now_register_send_cb(OnDataSent);
esp_now_register_recv_cb(OnDataRecv);
// ... existing code continues ...

// Modify sendSignalToStage2 function:
// Original line: Serial.println("ESP-NOW message sent: " + command);
// Replace with:
if(result == ESP_OK) {
  Serial.print("ESP-NOW message sent: ");
  Serial.println(command);
  if(strcmp(command, "startCycle") == 0) {
    waitingForAck = true;
    handshakeTimestamp = millis();
  }
} else {
  Serial.println("Error sending ESP-NOW message");
}

// In the loop() function under case READY, modify the new cycle trigger block to include handshake check:
// Find the READY state block and modify the if condition as follows:
if (currentState == READY) {
  // Check for handshake timeout and resend if necessary
  if (waitingForAck && (millis() - handshakeTimestamp > HANDSHAKE_TIMEOUT)) {
    Serial.println("Handshake timeout. Resending startCycle signal.");
    sendSignalToStage2("startCycle");
    handshakeTimestamp = millis();
  }

  // Only allow a new cycle if we are not waiting for an ack
  if (!waitingForAck && ((startCycleSwitch.rose() || (continuousModeActive && !cuttingCycleInProgress)) 
      && !woodSuctionError) && startSwitchSafe) {
    // ... existing cycle start code ...
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, HIGH);
    digitalWrite(BLUE_LED, LOW);
    cuttingCycleInProgress = true;
    currentState = CUTTING;
    cutMotor.setMaxSpeed(CUT_NORMAL_SPEED);
    cutMotor.setAcceleration(CUT_ACCELERATION);
    digitalWrite(POSITION_CLAMP, LOW);
    digitalWrite(WOOD_SECURE_CLAMP, LOW);
    if (!woodPresent) {
      digitalWrite(BLUE_LED, HIGH);
    }
  }
} 