#include "Arduino.h"
#include "SensorManager.h" 

void setup() {
  Serial.begin(115200);

  // Add setup for other serial ports, I2C, SPI, etc. if needed
 

  delay(2000); // Give Serial Monitor time to connect
  Serial.println(F("--- System Setup Starting ---"));

  setupPressureSensors();
  setupLoadCells();
  setupFlowSensors();
  setupTemperatureSensors();
  setupRelays();
  setupDCMotor();

  // Call setup functions for other sensor types here
  /*
  setupOtherSensors();
  */

  // Initialize the Serial Receive State Machine
  // currentRxState is a global variable defined in SensorManager.cpp
  currentRxState = RX_WAITING_FOR_START; // <-- Assignment only

  Serial.println(F("--- System Setup Complete. Starting loop ---"));

  // --- Run timing tests once after setup ---
  delay(1000); // Short delay before tests
  Serial.println(F("\n--- Running Initial Timing Tests ---"));

  // testTimingBatchAllTypes(); // Call the test function (defined in SensorManager.cpp)

  Serial.println(F("\n--- Initial Timing Tests Complete. Entering Main Loop ---"));
  delay(1000); // Delay before starting the continuous loop
}

void loop() {
  unsigned long currentMillis = millis(); // Get the current time
  unsigned long categoryDuration; // Declared ONCE at the top of loop() for reuse in all blocks

  // --- Serial Receive State Machine Block ---
  // This block runs every loop() iteration to process any incoming serial bytes
  while (Serial.available() > 0) {
    byte incomingByte = Serial.read(); // Read the next byte from the buffer

    switch (currentRxState) {
      case RX_WAITING_FOR_START:
        if (incomingByte == COMMAND_START_BYTE) {
          currentRxState = RX_READING_TYPE;
          // Reset variables for the new command packet
          rxCommandType = 0;
          rxTargetId = 0;
          rxPayloadSize = 0;
          rxPayloadBytesRead = 0;
          // No need to clear rxPayloadBuffer here, it will be overwritten
          // Serial.println("RX: Found Start"); // Debug
        }
        // If not the start byte, stay in this state and discard the byte.
        break;

      case RX_READING_TYPE:
        rxCommandType = incomingByte;
        currentRxState = RX_READING_TARGET_ID;
        // Serial.print("RX: Read Type: "); Serial.println(rxCommandType); // Debug
        break;

      case RX_READING_TARGET_ID:
        rxTargetId = incomingByte;
        currentRxState = RX_READING_SIZE;
        // Serial.print("RX: Read Target ID: "); Serial.println(rxTargetId); // Debug
        break;

      case RX_READING_SIZE:
        rxPayloadSize = incomingByte; // This is the expected size of JUST the payload
        rxPayloadBytesRead = 0; // Reset payload counter

        // Validate the received payload size against the expected maximum
        if (rxPayloadSize > MAX_COMMAND_PAYLOAD_SIZE) {
              Serial.print(F("RX Error: Payload size too large (")); Serial.print(rxPayloadSize); Serial.print(F("). Max is ")); Serial.print(MAX_COMMAND_PAYLOAD_SIZE); Serial.println(F(". Resetting."));
              currentRxState = RX_WAITING_FOR_START; // Discard packet
        } else if (rxPayloadSize == 0) {
              // If payload size is 0, we're done reading payload. Go straight to END.
              currentRxState = RX_READING_END;
              // Serial.println("RX: Read Size 0, going to End"); // Debug
        } else {
            // Expecting payload bytes next
            currentRxState = RX_READING_PAYLOAD;
            // Serial.print("RX: Read Size: "); Serial.println(rxPayloadSize); // Debug
        }
        break;

      case RX_READING_PAYLOAD:
        // Store the incoming byte in the buffer
        rxPayloadBuffer[rxPayloadBytesRead] = incomingByte;
        rxPayloadBytesRead++;

        // Check if we have read all the expected payload bytes
        if (rxPayloadBytesRead == rxPayloadSize) {
          currentRxState = RX_READING_END;
          // Serial.println("RX: Payload Complete, going to End"); // Debug
        }
        break;

      case RX_READING_END:
        // We expect the Command End Byte
        if (incomingByte == COMMAND_END_BYTE) {
          // --- COMMAND PACKET SUCCESSFULLY RECEIVED AND VALIDATED! ---
          // Process the command
          // Serial.println("RX: Found End. Processing command..."); // Debug
          handleCommand(rxCommandType, rxTargetId, rxPayloadBuffer, rxPayloadSize); // Pass the stored components
        } else {
          // Protocol error! Unexpected byte where END byte should be.
          Serial.print(F("RX Error: Expected End Byte (")); Serial.print(COMMAND_END_BYTE, HEX); Serial.print(F(") but got (")); Serial.print(incomingByte, HEX); Serial.println(F("). Resetting."));
          // Discard the corrupted packet.
        }

        // --- Reset the state for the next packet regardless of success or failure ---
        currentRxState = RX_WAITING_FOR_START;
        // Serial.println("RX: State Reset to WAITING_FOR_START"); // Debug
        break;
    } // End switch(currentRxState)
  } // End while(Serial.available())

  // --- NEW: Call the Startup Sequence Handler (Non-blocking) ---
  if (isStartupSequenceActive) {
    runStartupSequence();
  }

  // --- NEW: Call the Shutdown Sequence Handler (Non-blocking) ---
  if (isShutdownSequenceActive) {
    runShutdownSequence();
  }

// --- Pressure Sensors (BATCH PROCESSING - Runs every loop iteration) ---
pressureCategoryStartTime = micros(); // Assignment to global variable

for(int i = 0; i < NUM_PRESSURE_SENSORS; i++){ // Use local 'i' for loop counter
  unsigned long individualSensorStartTime = 0; // Local variable for individual sensor timing

  byte pressure_id = PRESSURE_ID_START + i; // Use 'i' for ID calculation
  startSensorTimer(pressure_id, &individualSensorStartTime); // Pass ID and address of local var

  // Perform sensor reading and calculation
  int raw_pressure_int = analogRead(PRESSURE_SENSOR_PINS[i]); // Use 'i' for pin access
  PressureSensorValues pressureData = calculatePressureSensorValues(raw_pressure_int, i); // Use 'i' for index

  // End timer for the individual sensor operation.
  // This function now *only* calculates the duration and stores it in pressureTimingData[i].
  // It no longer sends its own separate timing packet.
  endSensorTimer(pressure_id, individualSensorStartTime, "Pressure Sensor Block");

  // Retrieve the stored timing data for this specific sensor.
  // We get a pointer to the SensorTiming struct that 'endSensorTimer' just populated.
  const SensorTiming* currentSensorTiming = &pressureTimingData[i];

  // Send the pressure data in a binary packet, now including the timing data
  // as the optional last argument. The 'id' parameter is the sensor's ID.
  sendBinaryPacket(PRESSURE_PACKET_START_BYTE, pressure_id, &pressureData, sizeof(pressureData), PRESSURE_PACKET_END_BYTE, currentSensorTiming);
}
  
// End category timer AFTER the loop finishes for the entire batch
categoryDuration = micros() - pressureCategoryStartTime; // Calculate duration for the entire category batch
categoryTiming = {PRESSURE_ID_START, pressureCategoryStartTime, micros(), categoryDuration}; // Store category timing data
sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming); // Send the category timing packet separately  



  // --- Load Cells (BATCH PROCESSING - Runs every loop iteration) ---
  loadCellCategoryStartTime = micros(); // Assignment to global variable

  for(int i = 0; i < NUM_LOADCELL_SENSORS; i++){ // Use local 'i' for loop counter
    HX711& currentScale = scales[i]; // Use 'i' for array access

    if (currentScale.is_ready()) { // Check if data is available
      unsigned long individualSensorStartTime = 0; // Local variable for individual sensor timing
      
      byte loadCell_id = LOADCELL_ID_START + i; // Use 'i' for ID calculation
      startSensorTimer(loadCell_id, &individualSensorStartTime);

      // Perform sensor reading and calculation
      float raw_weight = currentScale.get_units(); // Might block briefly
      LoadCellValues loadCellData = calculateLoadCellValues(raw_weight);


      // End timer for the individual sensor operation
      endSensorTimer(loadCell_id, individualSensorStartTime, "Load Cell Block (read+calc+send)");

      // Send data
      const SensorTiming* currentSensorTiming = &loadCellTimingData[i];

      sendBinaryPacket(LOADCELL_PACKET_START_BYTE, loadCell_id, &loadCellData, sizeof(loadCellData), LOADCELL_PACKET_END_BYTE,currentSensorTiming);

    } else {
        // Serial.print(F("LoadCell ID ")); Serial.print(LOADCELL_ID_START + i); Serial.println(F(": Not ready.")); // Use 'i' for ID
      //add error handeling here but later
    }
  }

  // End category timer AFTER the loop finishes for the entire batch
  categoryDuration = micros() - loadCellCategoryStartTime; // Assignment only, no 'unsigned long'
  categoryTiming = {LOADCELL_ID_START, loadCellCategoryStartTime, micros(), categoryDuration}; // Assignment only, no 'SensorTiming'
  sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming);

  // Update lastProcessTime for consistency, though it no longer gates this loop
  lastLoadCellProcessTime = currentMillis;  
  
  // --- State Machine Logic for Flow Sensor ---
  // This block remains as originally provided by you.
  if (currentMillis - lastFlowProcessTime >= FLOW_CALCULATION_INTERVAL_MS) {
    unsigned long individualSensorStartTime = 0; // Local variable for individual sensor timing

    // For a single flow sensor, individual timing IS the category timing
    flowCategoryStartTime = micros(); // Assignment to global variable

    byte flow_id = FLOW_SENSOR_ID;
    startSensorTimer(flow_id, &individualSensorStartTime); // Start timer for the flow sensor's operation

    elapsed_time = currentMillis - lastFlowProcessTime;

    lastFlowProcessTime = currentMillis;

    long currentPulseCount;
    noInterrupts();
    currentPulseCount = flow_pulse;
    interrupts();

    long delta_pulse = currentPulseCount - flow_pulseLast;

    FlowMeterValues flowData = calculateFlowMeterValues(delta_pulse,elapsed_time);
    flow_pulseLast = currentPulseCount;

    // NEW: Store the calculated flowData globally for sequence access
    latestFlowMeterValues = flowData;

    // End timer for the flow sensor's operation
    endSensorTimer(flow_id, individualSensorStartTime, "Flow Sensor Block");
    const SensorTiming* currentSensorTiming = &flowTimingData[0];

    sendBinaryPacket(FLOW_PACKET_START_BYTE, flow_id, &flowData, sizeof(flowData), FLOW_PACKET_END_BYTE,currentSensorTiming);

    // End category timer (same as individual for single sensor)
    categoryDuration = micros() - flowCategoryStartTime; // Assignment only, no 'unsigned long'
    categoryTiming = {FLOW_SENSOR_ID, flowCategoryStartTime, micros(), categoryDuration}; // Assignment only, no 'SensorTiming'
    sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming);
  }

  // --- State Machine Logic for Temperature Sensors (MAX6675) ---
  // This block remains as originally provided by you.
  if (currentMillis - lastTempProcessTime >= MIN_TEMP_INTERVAL_MS) {
    unsigned long individualSensorStartTime = 0; // Local variable for individual sensor timing

    // Start category timer if this is the first sensor in the cycle
    if (currentTempSensorIndex == 0) {
        tempCategoryStartTime = micros(); // Assignment to global variable
    }

    // Start timer for the individual sensor operation
    byte temp_id = TEMP_ID_START + currentTempSensorIndex;
    startSensorTimer(temp_id, &individualSensorStartTime);

    // Perform sensor reading and calculation
    TemperatureSensorValues tempData = calculateTemperatureSensorValues(currentTempSensorIndex); // Cycles through 0, 1, 2, 3

    // Send data

    // End timer for the individual sensor operation
    endSensorTimer(temp_id, individualSensorStartTime, "Temp Sensor Block (incl. readCelsius wait)");

    const SensorTiming* currentSensorTiming = &tempTimingData[currentTempSensorIndex];

    // Send data
    sendBinaryPacket(TEMP_PACKET_START_BYTE, temp_id, &tempData, sizeof(tempData), TEMP_PACKET_END_BYTE,currentSensorTiming);

    currentTempSensorIndex++; // Will now cycle through 0, 1, 2, 3
    if (currentTempSensorIndex >= NUM_TEMP_SENSORS) {
      currentTempSensorIndex = 0; // Wrap around
      // End category timer when the last sensor in the category is processed
      categoryDuration = micros() - tempCategoryStartTime; // Assignment only, no 'unsigned long'
      categoryTiming = {TEMP_ID_START, tempCategoryStartTime, micros(), categoryDuration}; // Assignment only, no 'SensorTiming'
      sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming);
    }
    lastTempProcessTime = currentMillis;
  }

    // --- State Machine Logic for Motor RPM ---
    // This block remains as originally provided by you.
    if (currentMillis - lastMotorCalcTime >= MOTOR_CALCULATION_INTERVAL_MS) {
      unsigned long individualMotorStartTime = 0; // Local variable for individual sensor timing

      // For a single motor RPM sensor, individual timing IS the category timing
      motorCategoryStartTime = micros(); // Assignment to global variable

      byte motor_id = MOTOR_RPM_ID;
      startSensorTimer(motor_id, &individualMotorStartTime); // Use new timing function

      unsigned long interval_ms = currentMillis - lastMotorCalcTime; // Actual interval duration
      lastMotorCalcTime = currentMillis; // Update timer

      // --- Perform Motor RPM Calculation and Sending ---

      // 1. Safely read the current pulse count from the volatile variable
      unsigned long currentPulseCount;
      noInterrupts(); // Disable interrupts
      currentPulseCount = motor_pulse_count; // Read the volatile counter
      interrupts();    // Re-enable interrupts

      // 2. Calculate the number of pulses since the last check
      unsigned long delta_pulse = currentPulseCount - motor_last_pulse_count;

      // 3. Update motor_last_pulse_count for the next calculation interval
      motor_last_pulse_count = currentPulseCount;

      // 4. Calculate RPM
      MotorRPMValue mData = calculateMotorRPM(currentPulseCount, motor_last_pulse_count, interval_ms);

      endSensorTimer(motor_id, individualMotorStartTime, "Motor RPM Block");

      const SensorTiming* currentSensorTiming = &motorTimingData[0];

      
      // 5. Send the data using the GENERIC sender
      sendBinaryPacket(
        MOTOR_RPM_PACKET_START_BYTE, // Start byte for motor rpm packets
        MOTOR_RPM_ID,                // Unique ID for this motor RPM (14)
        &mData,                      // Pointer to the calculated data struct
        sizeof(mData),               // Size of the data struct
        MOTOR_RPM_PACKET_END_BYTE,   // End byte for motor rpm packets
        currentSensorTiming
      );


      // End category timer (same as individual for single sensor)
      categoryDuration = micros() - motorCategoryStartTime; // Assignment only, no 'unsigned long'
      categoryTiming = {MOTOR_RPM_ID, motorCategoryStartTime, micros(), categoryDuration}; // Assignment only, no 'SensorTiming'
      sendTimingPacket(TIMING_CATEGORY_CYCLE_ID, &categoryTiming);
    }
}
