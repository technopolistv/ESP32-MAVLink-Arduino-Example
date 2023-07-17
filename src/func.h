void init_wifi() {
  if (!WiFi.config(ip, gateway, subnet)) {
    Serial.println("Static IP configuration failed!");
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("\nConnecting to ");
  Serial.print(WIFI_SSID);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  Serial.print("\nConnected with IP address: ");
  Serial.println(WiFi.localIP());

  mavlink_udp.begin(udp_mavlink_port);
}

void send_mavlink(mavlink_message_t *mavlink_message) {
  uint8_t mavlink_message_buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t mavlink_message_length = mavlink_msg_to_send_buffer(mavlink_message_buffer, mavlink_message);

  mavlink_udp.beginPacket(groundstation_host, groundstation_port);
  mavlink_udp.write(mavlink_message_buffer, mavlink_message_length);
  mavlink_udp.endPacket();

  Serial.write(mavlink_message_buffer, mavlink_message_length);
}

void send_heartbeat() {
  mavlink_message_t mvl_tx_message;

  const int16_t mavlink_type = MAV_TYPE_GROUND_ROVER;
  const int16_t autopilot_type = MAV_AUTOPILOT_GENERIC;

  uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  uint8_t system_status = MAV_STATE_ACTIVE;
  uint32_t custom_mode = control_mode;

  if (armed) {
    base_mode |= MAV_MODE_MANUAL_ARMED;
  } else {
    base_mode |= MAV_MODE_MANUAL_DISARMED;
  }

  mavlink_msg_heartbeat_pack(system_id, component_id, &mvl_tx_message, mavlink_type, autopilot_type, base_mode, custom_mode, system_status);
  send_mavlink(&mvl_tx_message);
}

void send_systemstatus() {
  mavlink_sys_status_t sys_status;
  mavlink_message_t mvl_tx_message;

  sys_status.onboard_control_sensors_present = 0;
  sys_status.onboard_control_sensors_enabled = 0; 
  sys_status.onboard_control_sensors_health = 0;

  // generate random values for system status
  sys_status.load = random(500, 600);
  sys_status.voltage_battery = random(11000, 13000);
  sys_status.current_battery = random(1, 3);
  sys_status.battery_remaining = random(80, 100);
  
  sys_status.drop_rate_comm = 0;
  sys_status.errors_comm = 0;
  sys_status.errors_count1 = 0;
  sys_status.errors_count2 = 0;
  sys_status.errors_count3 = 0;
  sys_status.errors_count4 = 0;

  mavlink_msg_sys_status_encode(system_id, component_id, &mvl_tx_message, &sys_status);
  send_mavlink(&mvl_tx_message);
}

void send_radiostatus() {
  mavlink_radio_status_t radio_status;
  mavlink_message_t mvl_tx_message;

  radio_status.remrssi = WiFi.RSSI();
  radio_status.rssi = WiFi.RSSI();

  mavlink_msg_radio_status_encode(system_id, component_id, &mvl_tx_message, &radio_status);
  send_mavlink(&mvl_tx_message);
}


void send_position() {
  mavlink_global_position_int_t position;
  mavlink_message_t mvl_tx_message;

  // generate spoofed coordinates
  position.time_boot_ms = millis();
  position.lat = random(557515000, 557515100);
  position.lon = random(376158000, 376158100);
  position.alt = random(602900, 602950);
  position.hdg = random(0, 36000);

  mavlink_msg_global_position_int_encode(system_id, component_id, &mvl_tx_message, &position);
  send_mavlink(&mvl_tx_message);
}

void handle_message_command_long(mavlink_message_t *msg) {
  mavlink_command_long_t command_long;
  mavlink_command_ack_t mvl_command_ack;
  mavlink_message_t mvl_tx_message;

  mvl_command_ack.result = MAV_RESULT_FAILED;
  uint8_t result = MAV_RESULT_UNSUPPORTED;

  mavlink_msg_command_long_decode(msg, &command_long);

  switch (command_long.command) {
    case MAV_CMD_COMPONENT_ARM_DISARM: // LONG CMD ID 400 - https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
      mvl_command_ack.command = MAV_CMD_COMPONENT_ARM_DISARM;
      mvl_command_ack.result = MAV_RESULT_ACCEPTED;

      (command_long.param1 == 1) ? armed = 1 : armed = 0;

      mavlink_msg_command_ack_encode(system_id, component_id, &mvl_tx_message, &mvl_command_ack);
      send_mavlink(&mvl_tx_message);
      break;

    default:
      break;
  }
}

void handle_mission_manual_control(mavlink_message_t *msg) {
  mavlink_manual_control_t manual_control;

  mavlink_msg_manual_control_decode(msg, &manual_control);
  // pass &manual_control to motor control logic

  // uncomment below if you want to reply using the manual control message back to the groundstation (for debugging)  
  // mavlink_message_t mvl_tx_message;
  // mavlink_msg_manual_control_encode(system_id, component_id, &mvl_tx_message, &manual_control);
  // send_mavlink(&mvl_tx_message);
}

void parse_mavlink(uint8_t parsing_byte) {
  mavlink_message_t mvl_rx_message;
  mavlink_status_t status;

  if (mavlink_parse_char(MAVLINK_COMM_0, parsing_byte, &mvl_rx_message, &status)) {
    switch (mvl_rx_message.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT: // MSG ID 1 - https://mavlink.io/en/messages/common.html#HEARTBEAT
        previousTimeoutMillis = millis();

        if (armed) {
          digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
          digitalWrite(LED_RED, LOW);
        } else {
          digitalWrite(LED_RED, !digitalRead(LED_RED));
          digitalWrite(LED_GREEN, LOW);
        }
        break;

      case MAVLINK_MSG_ID_MANUAL_CONTROL: // MSG ID 69 - https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        handle_mission_manual_control(&mvl_rx_message);
        break;

      case MAVLINK_MSG_ID_COMMAND_LONG: // MSG ID 76 - https://mavlink.io/en/messages/common.html#COMMAND_LONG
        handle_message_command_long(&mvl_rx_message);
        break;

      default:
        break;
    }
  }
}