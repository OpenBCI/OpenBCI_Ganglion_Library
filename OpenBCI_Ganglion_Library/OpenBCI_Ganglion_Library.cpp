




/*
    OpenBCI Gagnlion Library
    Place the containing folder into your libraries folder inside the arduino folder in your Documents folder

    This library will work with a single OpenBCI Ganglion board

    Joel Murphy, Leif Percifield, AJ Keller, and Conor Russomanno made this.

*/

#include "OpenBCI_Ganglion_Library.h"
#include <ota_bootloader.h>

// CONSTRUCTOR
OpenBCI_Ganglion::OpenBCI_Ganglion(){

}

// <<<<<<<<<<<<<<<<<<<<<<<<<  BOARD WIDE FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


  void OpenBCI_Ganglion::initialize() {
    SPI.begin();
    SPI.setFrequency(4000);
    SPI.setDataMode(SPI_MODE0);
    pinMode(LIS2DH_SS,OUTPUT); digitalWrite(LIS2DH_SS,HIGH);
    pinMode(LIS_DRDY,INPUT_PULLDOWN);
    pinMode(MCP_DRDY,INPUT);
    pinMode(MCP_SS,OUTPUT); digitalWrite(MCP_SS,HIGH);
    pinMode(MCP_RST,OUTPUT); digitalWrite(MCP_RST,LOW);
    delay(100); digitalWrite(MCP_RST,HIGH);
    pinMode(DAC_SS,OUTPUT); digitalWrite(DAC_SS,HIGH);
    for(int i=0; i<5; i++){
      pinMode(impedanceSwitch[i],OUTPUT); digitalWrite(impedanceSwitch[i],LOW);
    }
    pinMode(LED, OUTPUT);
    digitalWrite(LED,LED_state);
    makeUniqueId(); // construct the name and serial number for advertisement
    SimbleeBLE_advdata = advdata;
    SimbleeBLE_advdata_len = sizeof(advdata);
    if (useSerial) {
      Serial.begin(115200);
    } else {
      Serial.begin(9600);
      SimbleeBLE.advertisementData = "Ganglion 1";
      SimbleeBLE.begin();
      SimbleeBLE.txPowerLevel = +4;
    }
    initSerialBuffer();
    startFromScratch(gain, sps);
    rssiTimer = millis();
  }


  void OpenBCI_Ganglion::makeUniqueId() {
    uint64_t id = getDeviceId();
    String stringy =  String(getDeviceIdLow(), HEX);
    advdata[11] = (uint8_t)stringy.charAt(0);
    advdata[12] = (uint8_t)stringy.charAt(1);
    advdata[13] = (uint8_t)stringy.charAt(2);
    advdata[14] = (uint8_t)stringy.charAt(3);
    SimbleeBLE.manufacturerName = "openbci.com";
    SimbleeBLE.modelNumber = "Ganglion";
    SimbleeBLE.hardwareRevision = "1.0.0";
    SimbleeBLE.softwareRevision = "1.1.1";
  }

  void OpenBCI_Ganglion::blinkLED() {
    // blue LED blinks when BLE is not connected
    if(!is_running && !BLEconnected){
      if(millis()-LED_timer > LED_DELAY_TIME){
        LED_timer = millis();
        LED_state = !LED_state;
        digitalWrite(LED,LED_state);
      }
    }
  }

  void OpenBCI_Ganglion::startFromScratch(unsigned long g, unsigned long s) {
    byte id;
    int ID;
    config_LIS2DH();
    config_MCP3912(g, s);
    updateDAC(DACmidline);  // place DAC into V/2 position
    loadString("OpenBCI Ganglion v",18,false); loadString((char*)SimbleeBLE.softwareRevision, 5, true);
    for (int i = 2; i <= advdata[0]; i++) {
      loadChar(advdata[i], false);
    }
    loadNewLine();
    loadString("LIS2DH ID: ",11,false); id = LIS2DH_read(WHO_AM_I); loadHex(id,1,true);
    loadString("MCP3912 CONFIG_1: ",18,false); digitalWrite(MCP_SS, LOW);
    MCP_sendCommand(CONFIG_1, MCP_READ); ID = MCP_readRegister();
    digitalWrite(MCP_SS, HIGH); loadHex(ID,3,true);
    loadNewLine();
    prepToSendBytes(); sendSerialBytesBlocking();

    loadString("send 'b' to start data stream", 29, true);
    loadString("send 's' to stop data stream", 28, true);
    loadString("use 1,2,3,4 to turn OFF channels", 32, true);
    loadString("use !,@,#,$ to turn ON channels", 31, true);
    loadString("send '?' to print all registers", 31, true);
    loadString("send 'v' to initialize board", 28, true);
    loadString("send '[' ']' to enable/disable synthetic square wave", 53, true);
    loadString("send 'z' 'Z' to start/stop impedance test", 42, true);
    loadString("send 'n','N' to enable/disable accelerometer", 45, true);
    prepToSendBytes();
    sampleCounter = 0xFF;
  }

  void OpenBCI_Ganglion::printAllRegisters_Serial() {
    if (!is_running) {
      LIS2DH_readAllRegs_Serial();
      MCP_readAllRegs();
    }
  }

  boolean OpenBCI_Ganglion::startRunning(void) {
    if (is_running == false) {
      is_running = true;
      sampleCounter = 0xFF;
      if (streamSynthetic) {
        for (int i = 1; i < 4; i++) {
          channelData[i] = 0;
        }
      }
      if (useSerial) {
        digitalWrite(LED, HIGH);
      }
      if (useAccel){ enable_LIS2DH(); }

      config_MCP3912(gain, sps);
      MCP_turnOnChannels();
      }
      return is_running;
    }

    void OpenBCI_Ganglion::processData() {
      MCP_dataReady = false;
      if(sampleCounter >= 201){ sampleCounter = 0; }
      if (streamSynthetic) {
        incrementSyntheticChannelData();
      } else {
        updateMCPdata();
        if (useAccel) updateAccelerometerData();
      }
      if (sampleCounter == 0) {
        buildRawPacket();   // assemble raw packet on sampleCounter roll-over
        sendRawPacket();    // send raw sample packet
      } else {
        if(useAccel == true){
          if(accelOnEdge == true){
            accelOnEdge = false;
            if(sampleCounter%2 == 0){  compressData19(); return; }
          }
          compressData18();     // compress deltas and send on even samples
        } else {
          if(accelOffEdge == true){
            accelOffEdge = false;
            if(sampleCounter%2 == 0){  compressData18(); return; }
          }
          compressData19();     // compress deltas and send on even samples
        }
      }

    }

    boolean OpenBCI_Ganglion::stopRunning(void) {
      if (is_running == true) {
        is_running = false;
        MCP_turnOffAllChannels();
        disable_LIS2DH();
      }
      streamSynthetic = false;
      return is_running;
    }



    int OpenBCI_Ganglion::changeChannelState_maintainRunningState(int chan, int start) {
      boolean was_running_when_called = is_running;
      //must stop running to turn channel on/off
      stopRunning();
      if (start == 1) {
        if(!was_running_when_called){
          loadString("Activating channel ", 19, false); loadInt(chan, true);
        }
        channelMask &= channelEnable[chan - 1]; // turn on the channel
      } else {
        if(!was_running_when_called){
          loadString("Deactivating channel ", 21, false); loadInt(chan, true);
        }
        channelMask |= channelDisable[chan - 1]; // turn off the channel
      }
      if (!was_running_when_called){
        prepToSendBytes();
      } else {
        startRunning();
      }
    }

    void OpenBCI_Ganglion::initSyntheticData() {
      for (int i = 1; i < 4; i++) {
        channelData[i] = 0;
      }
    }

    void OpenBCI_Ganglion::startRunningSynthetic() {
      thatTime = millis();
      startRunning();
    }

    void OpenBCI_Ganglion::incrementSyntheticChannelData() {
      thisTime = millis();
      if (thisTime - thatTime > syntheticFrequency) {
        thatTime = thisTime;
        for (int i = 0; i < 4; i++) {
          rising[i] = !rising[i];
          if (rising[i]) {
            channelData[i] = 8000;
          } else {
            channelData[i] = -8000;
          }
        }
      }
    }

    void OpenBCI_Ganglion::buildRawPacket() {
      int byteCounter = 0;
      ringBufferLevel = 0;
      for (int i = 0; i < 4; i++) {
        for (int j = 16; j >= 0; j -= 8) { // fill the raw data array for streaming
          compression_ring[ringBufferLevel][byteCounter] = ((channelData[i] >> j) & 0xFF);
          byteCounter++;
        }
        lastChannelData[i] = channelData[i];  // keep track of the previous value
      }
    }


    void OpenBCI_Ganglion::sendRawPacket() {
      radioBuffer[0] = ringBufferLevel;
      for (int i = 0; i < 12; i++) {
        radioBuffer[i + 1] = compression_ring[ringBufferLevel][i];
      }
      for (char c = 0; c < 7; c++) {
        radioBuffer[c + 13] = c + 'A'; // padding
      }
      ringBufferLevel++;

      if (BLEconnected) {
        SimbleeBLE.send(radioBuffer, 20);
      }
    }
    /*
        send two samples in one packet by compressing the data
        first, take the delta between the current sample and the last sample
        then, retain the sign of the delta by moving it to bit 0
        then, only send the LSBs
        this function crams the data into a BLE packet
    */
    void OpenBCI_Ganglion::compressData18() {
      int deltas[4];
      int even = sampleCounter % 2;
      for (int i = 0; i < 4; i++) {
        deltas[i] = lastChannelData[i] - channelData[i];  // subtract new from old
        lastChannelData[i] = channelData[i];  // keep track of the previous value
        bitWrite(deltas[i], 0, (bitRead(deltas[i], 31))); // store the sign bit in bit0
      }
      if (even == 1) { // pack odd samples first
        compression_ring[ringBufferLevel][0] = ((deltas[0] &  0x0003FC00) >> 10);
        compression_ring[ringBufferLevel][1] = ((deltas[0] &  0x000003FC) >> 2);
        compression_ring[ringBufferLevel][2] = ((deltas[0] &  0x00000003) << 6);
        compression_ring[ringBufferLevel][2] |= ((deltas[1] & 0x0003F000) >> 12);
        compression_ring[ringBufferLevel][3] = ((deltas[1] &  0x00000FF0) >> 4);
        compression_ring[ringBufferLevel][4] = ((deltas[1] &  0x0000000F) << 4);
        compression_ring[ringBufferLevel][4] |= ((deltas[2] & 0x0003C000) >> 14);
        compression_ring[ringBufferLevel][5] = ((deltas[2] &  0x00003FC0) >> 6);
        compression_ring[ringBufferLevel][6] = ((deltas[2] &  0x0000003F) << 2);
        compression_ring[ringBufferLevel][6] |= ((deltas[3] & 0x00030000) >> 16);
        compression_ring[ringBufferLevel][7] = ((deltas[3] &  0x0000FF00) >> 8);
        compression_ring[ringBufferLevel][8] = (deltas[3] &   0x000000FF);
      } else {       // pack even samples second
        compression_ring[ringBufferLevel][9] = ((deltas[0] &   0x0003FC00) >> 10);
        compression_ring[ringBufferLevel][10] = ((deltas[0] &  0x000003FC) >> 2);
        compression_ring[ringBufferLevel][11] = ((deltas[0] &  0x00000003) << 6);
        compression_ring[ringBufferLevel][11] |= ((deltas[1] & 0x0003F000) >> 12);
        compression_ring[ringBufferLevel][12] = ((deltas[1] &  0x00000FF0) >> 4);
        compression_ring[ringBufferLevel][13] = ((deltas[1] &  0x0000000F) << 4);
        compression_ring[ringBufferLevel][13] |= ((deltas[2] & 0x0003C000) >> 14);
        compression_ring[ringBufferLevel][14] = ((deltas[2] &  0x00003FC0) >> 6);
        compression_ring[ringBufferLevel][15] = ((deltas[2] &  0x0000003F) << 2);
        compression_ring[ringBufferLevel][15] |= ((deltas[3] & 0x00030000) >> 16);
        compression_ring[ringBufferLevel][16] = ((deltas[3] &  0x0000FF00) >> 8);
        compression_ring[ringBufferLevel][17] = (deltas[3] &   0x000000FF);

        sendCompressedPacket18(); // send on the even packet
      }
    }

    void OpenBCI_Ganglion::sendCompressedPacket18() {
      radioBuffer[0] = ringBufferLevel;
      for (int i = 0; i < 18; i++) {
        radioBuffer[i+1] = compression_ring[ringBufferLevel][i];
      }
      if(useAccel){
        if(ringBufferLevel%10 == 1){ radioBuffer[19] = axisData[0]; }
        if(ringBufferLevel%10 == 2){ radioBuffer[19] = axisData[1]; }
        if(ringBufferLevel%10 == 3){ radioBuffer[19] = axisData[2]; }
      } else if(useAux){
        if(ringBufferLevel%10 == 1){ radioBuffer[19] = auxData[0]; }
        if(ringBufferLevel%10 == 2){ radioBuffer[19] = auxData[1]; }
        if(ringBufferLevel%10 == 3){ radioBuffer[19] = auxData[2]; }
      }
      ringBufferLevel++;
      if (BLEconnected) {
        SimbleeBLE.send(radioBuffer, 20);
      }
    }


    void OpenBCI_Ganglion::compressData19() {
      int deltas[4];
      int even = sampleCounter % 2;
      for (int i = 0; i < 4; i++) {
        deltas[i] = lastChannelData[i] - channelData[i];  // subtract new from old
        lastChannelData[i] = channelData[i];  // keep track of the previous value
        bitWrite(deltas[i], 0, (bitRead(deltas[i], 31))); // store the sign bit in bit0
      }
      if (even == 1) { // pack odd samples first
        compression_ring[ringBufferLevel][0] = ((deltas[0] & 0x0007F800) >> 11);
        compression_ring[ringBufferLevel][1] = ((deltas[0] & 0x000007F8) >> 3);
        compression_ring[ringBufferLevel][2] = ((deltas[0] & 0x00000007) << 5);
        compression_ring[ringBufferLevel][2] |= ((deltas[1] & 0x0007C000) >> 14);
        compression_ring[ringBufferLevel][3] = ((deltas[1] & 0x00003FC0) >> 6);
        compression_ring[ringBufferLevel][4] = ((deltas[1] & 0x0000003F) << 2);
        compression_ring[ringBufferLevel][4] |= ((deltas[2] & 0x00060000) >> 17);
        compression_ring[ringBufferLevel][5] = ((deltas[2] & 0x0001FE00) >> 9);
        compression_ring[ringBufferLevel][6] = ((deltas[2] & 0x000001FE) >> 1);
        compression_ring[ringBufferLevel][7] = ((deltas[2] & 00000001) << 7);
        compression_ring[ringBufferLevel][7] |= ((deltas[3] & 0x0007F000) >> 12);
        compression_ring[ringBufferLevel][8] = ((deltas[3] & 0x00000FF0) >> 4);
        compression_ring[ringBufferLevel][9] = ((deltas[3] & 0x0000000F) << 4);
      } else {       // pack even samples second
        compression_ring[ringBufferLevel][9] |= ((deltas[0] & 0x00078000) >> 15);
        compression_ring[ringBufferLevel][10] = ((deltas[0] & 0x00007F80) >> 7);
        compression_ring[ringBufferLevel][11] = ((deltas[0] & 0x0000007F) << 1);
        compression_ring[ringBufferLevel][11] |= ((deltas[1] & 0x00040000) >> 18);
        compression_ring[ringBufferLevel][12] = ((deltas[1] & 0x0003FC00) >> 10);
        compression_ring[ringBufferLevel][13] = ((deltas[1] & 0x000003FC) >> 2);
        compression_ring[ringBufferLevel][14] = ((deltas[1] & 0x00000003) << 6);
        compression_ring[ringBufferLevel][14] |= ((deltas[2] & 0x0007E000) >> 13);
        compression_ring[ringBufferLevel][15] = ((deltas[2] & 0x00001FE0) >> 5);
        compression_ring[ringBufferLevel][16] = ((deltas[2] & 0x0000001F) << 3);
        compression_ring[ringBufferLevel][16] |= ((deltas[3] & 0x00070000) >> 16);
        compression_ring[ringBufferLevel][17] = ((deltas[3] & 0x0000FF00) >> 8);
        compression_ring[ringBufferLevel][18] = (deltas[3] & 0x000000FF);

        sendCompressedPacket19(); // send on the even packet
      }
    }



    void OpenBCI_Ganglion::sendCompressedPacket19() {
      radioBuffer[0] = ringBufferLevel + 100;
      for (int i = 0; i < 19; i++) {
        radioBuffer[i+1] = compression_ring[ringBufferLevel][i];
      }
      ringBufferLevel++;
      if (BLEconnected) {
        SimbleeBLE.send(radioBuffer, 20);
      }
    }


    void OpenBCI_Ganglion::testImpedance() {
      if (!ACwaveTest) {
        uAsampleCounter = 0;
        negativeRunningTotal = positiveRunningTotal = positiveSampleCounter = negativeSampleCounter = 0;
        changeZtestForChannel(channelUnderZtest, 1); // trun on the current for testing
        realZeroPosition = getDACzeroPosition();
        ACwaveTest = true;
        ACrising = false;
        int runningTotal = 0;
        for (int i = 0; i < 10; i++) {
          runningTotal += analogRead(SHUNT_SENSOR);
          delay(1);
        }
        currentCountsAtZeroAmps = (runningTotal / 10);
        maxPosCurrentCounts = minNegCurrentCounts = 0;
        updateDAC(realZeroPosition - HALF_WAVE);
        halfPeriodTimer = uAsampleTimer = micros();
      }
      else
      {
        thisTime = micros();  // time critical activities!
        if (thisTime - halfPeriodTimer > HALF_PERIOD) {
          halfPeriodTimer = thisTime;
          if (ACrising) {
            updateDAC(realZeroPosition - HALF_WAVE);
            ACrising = false;
            edge = true;
            uAsampleTimer = thisTime;
          } else {
            updateDAC(realZeroPosition + HALF_WAVE);
            ACrising = true;
            edge = true;
            uAsampleTimer = thisTime;
          }
        }
        if (thisTime - uAsampleTimer > UA_SAMPLE_TIME) {
          if (uAsampleCounter > UA_SAMPLE_LIMIT) {
            updateDAC(realZeroPosition);
            ACwaveTest = false;
            positiveMean = positiveRunningTotal / positiveSampleCounter;
            negativeMean = negativeRunningTotal / negativeSampleCounter;
            changeZtestForChannel(channelUnderZtest, 0); // turn off the current switch
                  //  Serial.println("* Test Complete");
            // int impedance = (HALF_WAVE * 2 * DAC_volts_per_count) / ((((maxPosCurrentCounts - minNegCurrentCounts) * ADC_volts_per_count)/SHUNT_SENSOR_GAIN)/(SHUNT));///2));
            int _impedance = (HALF_WAVE * DAC_volts_per_count) / (((((maxPosCurrentCounts - minNegCurrentCounts)/2) * ADC_volts_per_count)/SHUNT_SENSOR_GAIN)/SHUNT);
                  // Serial.print("positiveMax = "); Serial.println(maxPosCurrentCounts);
                  // Serial.print("negativeMin = "); Serial.println(minNegCurrentCounts);
                  // Serial.print("_impedance "); Serial.println(_impedance);
            double _imp = double(_impedance)/1000.0;
            double impedance = convertRawGanglionImpedanceToTarget(_imp);
            initSerialBuffer();
            loadInt(impedance, false); loadChar('Z', true);
            serialBuffer[0][0] = ID_Z_1 + (channelUnderZtest - 1);
            timeLastPacketSent = millis();  // prime the timer to send verbose packets
            bufferLevelCounter = 0;
            serialBytesToSend = true; sendSerialBytesBlocking();
            channelUnderZtest++;
            if (channelUnderZtest > 5) { channelUnderZtest = 1; }
            return;
          }

          currentCounts = (analogRead(SHUNT_SENSOR) - currentCountsAtZeroAmps);
              //  Serial.println(currentCounts);  // verbose feedback
          if (ACrising) {
            if(currentCounts > maxPosCurrentCounts){ maxPosCurrentCounts = currentCounts; }
          } else {
            if(currentCounts < minNegCurrentCounts){ minNegCurrentCounts = currentCounts; }
          }
          uAsampleTimer = thisTime;
          uAsampleCounter++;
        }
      }
    }

    void OpenBCI_Ganglion::changeZtestForChannel(int channel, int setting) {
      digitalWrite(impedanceSwitch[channel - 1], setting);
    }

    void OpenBCI_Ganglion::endImpedanceTest(){
      testingImpedance = false;
      updateDAC(realZeroPosition);
      for(int i=1; i<=5; i++){
        changeZtestForChannel(i, 0);
      }
    }

    double OpenBCI_Ganglion::convertRawGanglionImpedanceToTarget(double _actual){
      //the following impedance adjustment calculations were derived using empirical values from resistors between 1,2,3,4,REF-->D_G
      double _target;
      //V1 -- more accurate for lower impedances (< 22kOhcm) -> y = 0.0034x^3 - 0.1443x^2 + 3.1324x - 10.59
      if(_actual <= 22.0){
        // _target = (0.0004)*(pow(_actual,3)) - (0.0262)*(pow(_actual,2)) + (1.8349)*(_actual) - 6.6006;
        _target = (0.0034)*(pow(_actual,3)) - (0.1443)*(pow(_actual,2)) + (3.1324)*(_actual) - 10.59;
      }
      //V2 -- more accurate for higher impedances (> 22kOhm) -> y = 0.000009x^4 - 0.001x^3 + 0.0409x^2 + 0.6445x - 1
      else {
        _target = (0.000009)*(pow(_actual,4)) - (0.001)*pow(_actual,3) + (0.0409)*(pow(_actual,2)) + (0.6445)*(pow(_actual,1)) - 1;
      }

      return _target;

    }



    // <<<<<<<<<<<<<<<<<<<<<<<<<  END OF BOARD WIDE FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // *************************************************************************************
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  AD5621 DAC FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    void OpenBCI_Ganglion::updateDAC(word DAC_pos) { //
      SPI.setDataMode(SPI_MODE1);  // DAC uses MODE 1

      word command = (DAC_pos << 2) & DAC_MASK;
      byte highCommand = (command >> 8) & 0xFF;
      byte lowCommand = command & 0xFF;
      digitalWrite(DAC_SS, LOW);
      SPI.transfer(highCommand);
      SPI.transfer(lowCommand);
      digitalWrite(DAC_SS, HIGH);
      DAC_position = DAC_pos;

      SPI.setDataMode(SPI_MODE0); // Everything else uses MODE 0
    }

    void OpenBCI_Ganglion::updateDAC() {
      SPI.setDataMode(SPI_MODE1);  // DAC uses MODE 1

      word command = (DAC_position << 2) & DAC_MASK;
      digitalWrite(DAC_SS, LOW);
      SPI.transfer(highByte(command));
      SPI.transfer(lowByte(command));
      digitalWrite(DAC_SS, HIGH);

      SPI.setDataMode(SPI_MODE0); // Everything else uses MODE 0
    }

    void OpenBCI_Ganglion::zeroDAC() {
      SPI.setDataMode(SPI_MODE1);  // DAC uses MODE 1

      word command = (DAC_position << 2) & DAC_MASK;
      command |= DAC_1K;
      digitalWrite(DAC_SS, LOW);
      SPI.transfer(highByte(command));
      SPI.transfer(lowByte(command));
      digitalWrite(DAC_SS, HIGH);

      SPI.setDataMode(SPI_MODE0); // Everything else uses MODE 0
    }

    float OpenBCI_Ganglion::get_Zvalue(int DAC_pos) {
      DAC_voltage = (DAC_pos * DAC_volts_per_count);  // - 1.5;
      Ohms = int(DAC_voltage / 0.00001);
      return Ohms;
    }


    word OpenBCI_Ganglion::getDACzeroPosition() {
      updateDAC(DACmidline);
      sampleNumber = 0;
      sampleTimer = micros();
      while (sampleNumber < 100) {
        gotoTarget(0.0, noise);
      }
      return DAC_position;
    }

    void OpenBCI_Ganglion::readShuntSensor() {
      currentCounts = analogRead(SHUNT_SENSOR);
      //  nAmp_Value = currentCounts * nAmps_per_count;
      uAmp_Value = float(currentCounts) * ADC_volts_per_count;
      // uAmp_Value -= 15.0;  //
      uAmp_Value -= 1.5;
    }

    void OpenBCI_Ganglion::gotoTarget(float target, float n) {
      if (micros() - sampleTimer > gotoSampleTime) {
        sampleTimer = micros();
        sampleNumber++;
        readShuntSensor();
        if (uAmp_Value > target + n) {
          decreased++;
          DAC_position--; updateDAC();
        } else if (uAmp_Value < target - n) {
          increased++;
          DAC_position++; updateDAC();
        } else {
          steady++;
        }
      }
    }



    // <<<<<<<<<<<<<<<<<<<<<<<<<  END OF AD5621 DAC FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // *************************************************************************************
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  LIS2DH FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



    void OpenBCI_Ganglion::updateAccelerometerData() {
       if(digitalRead(LIS_DRDY) == HIGH){
         axisData[0] = LIS2DH_read(OUT_X_H); // read out the 8bit axis values
         axisData[1] = LIS2DH_read(OUT_Y_H);
         axisData[2] = LIS2DH_read(OUT_Z_H);
         newAccelData = true;
       }

    }

    void OpenBCI_Ganglion::config_LIS2DH() {
      LIS2DH_write(TEMP_CFG_REG, 0xC0);  // enable temperature sensor
      LIS2DH_write(CTRL_REG1, 0x28);     // 10Hz data rate, low-power mode, axis disabled
      LIS2DH_write(CTRL_REG3, 0x10);     // DRDY1 INTERUPT ON INT_1 PIN
      LIS2DH_write(CTRL_REG4, 0x00);     // 0x10 = +/- 4G, 0x00 = +/- 2G axis data continuous update

    }

    void OpenBCI_Ganglion::enable_LIS2DH() {
      LIS2DH_write(CTRL_REG1, 0x2F); // 10Hz data rate, low power mode, axis enabled
    }

    void OpenBCI_Ganglion::disable_LIS2DH() {
      LIS2DH_write(CTRL_REG1, 0x28); // 10Hz data rate, low-power mode, axis disabled
    }

    word OpenBCI_Ganglion::LIS2DH_readTemp() {
      word temp = 0;
      if ((LIS2DH_read(STATUS_REG_AUX) & 0x04) > 1) { // check for updated temp data...
        temp = LIS2DH_read16(OUT_TEMP_L);
        if (!is_running || BLEconnected) {
          loadString("Temperature ", 12, false); loadInt(temp, false); loadString("*", 1, true); // 12, 1
          prepToSendBytes();
        }
      }
      return temp;
    }

    byte OpenBCI_Ganglion::LIS2DH_read(byte reg) {
      reg |= READ_REG;
      digitalWrite(LIS2DH_SS, LOW);
      SPI.transfer(reg);
      byte inByte = SPI.transfer(0x00);
      digitalWrite(LIS2DH_SS, HIGH);
      return inByte;
    }


    void OpenBCI_Ganglion::LIS2DH_write(byte reg, byte value) {
      digitalWrite(LIS2DH_SS, LOW);
      SPI.transfer(reg);
      SPI.transfer(value);
      digitalWrite(LIS2DH_SS, HIGH);
    }

    short OpenBCI_Ganglion::LIS2DH_read16(byte reg) {
      short inData;
      reg |= READ_REG | READ_MULTI;
      digitalWrite(LIS2DH_SS, LOW);
      SPI.transfer(reg);
      inData = SPI.transfer(0x00) | (SPI.transfer(0x00) << 8);
      digitalWrite(LIS2DH_SS, HIGH);
      return inData;
    }

    float OpenBCI_Ganglion::getG(byte axis) {
      short counts = LIS2DH_read16(axis);
      float gValue = float(counts) * scale_factor_gs_per_count;
      return gValue;
    }

    void OpenBCI_Ganglion::LIS2DH_readAllRegs_Serial() {
      loadString("LIS2DH\nREG\tSetting", 18, true);
      byte inByte;
      byte reg = STATUS_REG_AUX | READ_REG;
      digitalWrite(LIS2DH_SS, LOW);
      SPI.transfer(reg);
      inByte = SPI.transfer(0x00);
      digitalWrite(LIS2DH_SS, HIGH);
      loadHex(reg & 0x7F, 1, false);
      loadString("\t", 1, false); loadHex(inByte, 1, true);
      digitalWrite(LIS2DH_SS, HIGH);
      reg = OUT_TEMP_L | READ_REG | READ_MULTI;
      digitalWrite(LIS2DH_SS, LOW);
      SPI.transfer(reg);
      for (int i = OUT_TEMP_L; i <= WHO_AM_I; i++) {
        inByte = SPI.transfer(0x00);
        loadHex(i, 1, false);
        loadChar('\t', false); loadHex(inByte, 1, true);
      }
      digitalWrite(LIS2DH_SS, HIGH);
      prepToSendBytes();
      sendSerialBytesBlocking();
      reg = TEMP_CFG_REG | READ_REG | READ_MULTI;
      digitalWrite(LIS2DH_SS, LOW);
      SPI.transfer(reg);
      for (int i = TEMP_CFG_REG; i <= ACT_DUR; i++) {
        inByte = SPI.transfer(0x00);
        loadHex(i, 1, false);
        loadChar('\t', false); loadHex(inByte, 1, true);
      }
      digitalWrite(LIS2DH_SS, HIGH);
      loadNewLine();
      prepToSendBytes();
      sendSerialBytesBlocking();
    }

    // <<<<<<<<<<<<<<<<<<<<<<<<<  END OF LIS2DH FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // *************************************************************************************
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  MCP3912 FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


    void OpenBCI_Ganglion::config_MCP3912(unsigned long gain, unsigned long sampleRate) {
      sampleRate |= 0x003CE050; // dither on max, boost 2x, OSR 4096,
      // digitalWrite(MCP_RST, LOW); delay(50);
      // digitalWrite(MCP_RST, HIGH); delay(300);
      digitalWrite(MCP_SS, LOW);
      MCP_sendCommand(GAIN, MCP_WRITE);
      MCP_writeRegister(gain);          // GAIN_1, _2, _4, _8, _16, _32
      MCP_writeRegister(0x00B9000F);    // STATUSCOM auto increment TYPES DR in HIZ
      MCP_writeRegister(sampleRate);    // CONFIG_0:  0x003CE050 | sample rate: 50, 100, 200, 400
      MCP_writeRegister(0x000F0000);    // CONFIG_1:  put the ADCs in reset, external oscillator
      digitalWrite(MCP_SS, HIGH);
    }

    void OpenBCI_Ganglion::updateMCPdata() {
      // int byteCounter = 0;
      digitalWrite(MCP_SS, LOW);
      MCP_sendCommand(channelAddress[0], MCP_READ); // send request to read from CHAN_0 address
      for (int i = 0; i < 4; i++) {
        channelData[i] = MCP_readRegister();  // read the 24bit result into the long variable array
        // for (int j = 16; j >= 0; j -= 8) {
        //   rawChannelData[byteCounter] = (channelData[i] >> j & 0xFF); // fill the raw data array for streaming
        //   byteCounter++;
        // }
      }
      digitalWrite(MCP_SS, HIGH);
      // this section corrects the sign on the long array
      for (int i = 0; i < 4; i++) {
        if ((channelData[i] & 0x00800000) > 0) {
          channelData[i] |= 0xFF000000;
        } else {
          channelData[i] &= 0x00FFFFFF;
        }
      }
    }

    void OpenBCI_Ganglion::MCP_sendCommand(byte address, byte rw) {
      byte command = DEV_ADD | address | rw;
      SPI.transfer(command);
    }


    long OpenBCI_Ganglion::MCP_readRegister() {

      long thisRegister = SPI.transfer(0x00);
      thisRegister <<= 8;
      thisRegister |= SPI.transfer(0x00);
      thisRegister <<= 8;
      thisRegister |= SPI.transfer(0x00);

      return thisRegister;
    }

    void OpenBCI_Ganglion::MCP_writeRegister(unsigned long setting) {
      byte thisByte = (setting & 0x00FF0000) >> 16;
      SPI.transfer(thisByte);
      thisByte = (setting & 0x0000FF00) >> 8;
      SPI.transfer(thisByte);
      thisByte = setting & 0x000000FF;
      SPI.transfer(thisByte);
    }


    void OpenBCI_Ganglion::MCP_turnOnChannels() {
      digitalWrite(MCP_SS, LOW);
      MCP_sendCommand(CONFIG_1, MCP_WRITE);
      MCP_writeRegister(channelMask);  // turn on selected channels
      digitalWrite(MCP_SS, HIGH);

    }

    void OpenBCI_Ganglion::MCP_turnOffAllChannels() {
      digitalWrite(MCP_SS, LOW);
      MCP_sendCommand(CONFIG_1, MCP_WRITE);
      MCP_writeRegister(0x000F0000);  // turn off all channels
      digitalWrite(MCP_SS, HIGH);
    }


    void OpenBCI_Ganglion::MCP_readAllRegs() {
      loadString("MCP3912\nREG\tSetting", 19, true);
      for (int i = MOD_VAL; i <= GAINCAL_3; i += 2) {
        if (i != 0x12) {
          digitalWrite(MCP_SS, LOW);
          MCP_sendCommand(i, MCP_READ);
          regVal = MCP_readRegister();
          digitalWrite(MCP_SS, HIGH);
          MCP_printRegisterName(i);
          loadHex(regVal, 3, true);
        }
      }
      digitalWrite(MCP_SS, LOW);
      prepToSendBytes();
      sendSerialBytesBlocking();
      delay(10);
      MCP_sendCommand(LOK_CRC, MCP_READ);
      regVal = MCP_readRegister();
      digitalWrite(MCP_SS, HIGH);
      MCP_printRegisterName(LOK_CRC);
      loadHex(regVal, 3, true);
      prepToSendBytes();
      sendSerialBytesBlocking();
    }

    void OpenBCI_Ganglion::MCP_printRegisterName(byte _address) {

      switch (_address) {
        case MOD_VAL:
          loadString("MOD_VAL   ", 10, false); break;
        case GAIN:
          loadString("GAIN      ", 10, false); break;
        case PHASE:
          loadString("PHASE     ", 10, false); break;
        case STATUSCOM:
          loadString("STATUSCOM ", 10, false); break;
        case CONFIG_0:
          loadString("CONFIG_0  ", 10, false); break;
        case CONFIG_1:
          loadString("CONFIG_1  ", 10, false); break;
        case OFFCAL_0:
          loadString("OFFCAL_0  ", 10, false); break;
        case GAINCAL_0:
          loadString("GAINCAL_0 ", 10, false); break;
        case OFFCAL_1:
          loadString("OFFCAL_1  ", 10, false); break;
        case GAINCAL_1:
          loadString("GAINCAL_1 ", 10, false); break;
        case OFFCAL_2:
          loadString("OFFCAL_2  ", 10, false); break;
        case GAINCAL_2:
          loadString("GAINCAL_2 ", 10, false); break;
        case OFFCAL_3:
          loadString("OFFCAL_3  ", 10, false); break;
        case GAINCAL_3:
          loadString("GAINCAL_3 ", 10, false); break;
        case LOK_CRC:
          loadString("LOK_CRC   ", 10, false); break;
        default:
          break;
      }

    }


    // <<<<<<<<<<<<<<<<<<<<<<<<<  END OF MCP3912 FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // *************************************************************************************
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<  COM FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>




    boolean OpenBCI_Ganglion::eventSerial() {

      if(clearForOTA){
        clearForOTA = false;
        delay(100);
        ota_bootloader_start(); //begins OTA enabled state
      }

      if(gotBLE){
        gotBLE = false;
        parseChar(BLEchar);
      }

      while (Serial.available()) {
        inChar = Serial.read();
        gotSerial = true;
        parseChar(inChar);
      }

      if (serialBytesToSend) {
        // send via BLE if available
        if (BLEconnected) {
          if ((millis() - timeLastPacketSent) > 15) {
            SimbleeBLE.send(serialBuffer[bufferLevelCounter], serialIndex[bufferLevelCounter]);
            bufferLevelCounter++;        // get ready for next buffered packet
            if (bufferLevelCounter == bufferLevel + 1) { // when we send all the packets
              serialBytesToSend = false;                    // put down bufferToSend flag
              bufferLevel = 0;                        // initialize bufferLevel
              initSerialBuffer();                     // initialize bufffer
            }
            timeLastPacketSent = millis();
          }
        } else {
          // send via Serial Port if no BLE available
          for (int i = 0; i <= bufferLevel; i++) {
            for (int j = 1; j < serialIndex[i]; j++) {
              Serial.write(serialBuffer[i][j]);
            }
          }
          serialBytesToSend = false;                    // put down bufferToSend flag
          bufferLevel = 0;                        // initialize bufferLevel
          initSerialBuffer();                     // initialize bufffer
        }
      }

      return gotSerial;
    }


    void OpenBCI_Ganglion::sendSerialBytesBlocking() {
      // send via BLE if available
      if (BLEconnected) {
        while (serialBytesToSend) {
          if ((millis() - timeLastPacketSent) > 15) {
            SimbleeBLE.send(serialBuffer[bufferLevelCounter], serialIndex[bufferLevelCounter]);
            bufferLevelCounter++;        // get ready for next buffered packet
            if (bufferLevelCounter == bufferLevel + 1) { // when we send all the packets
              //          Serial.println(serialBuffer[0][0]);
              serialBytesToSend = false;                    // put down bufferToSend flag
              bufferLevel = 0;                        // initialize bufferLevel
              initSerialBuffer();                     // initialize bufffer
            }
            timeLastPacketSent = millis();
          }
        }
      } else {
        // send via Serial Port if no BLE available
        for (int i = 0; i <= bufferLevel; i++) {
          for (int j = 1; j < serialIndex[i]; j++) {
            Serial.write(serialBuffer[i][j]);
          }
        }
        serialBytesToSend = false;                    // put down bufferToSend flag
        bufferLevel = 0;                        // initialize bufferLevel
        initSerialBuffer();                     // initialize bufffer
      }
    }

    void OpenBCI_Ganglion::prepToSendBytes() {
      if (serialIndex[bufferLevel] == 0) {
        bufferLevel--; // don't send an empty buffer!
      }
      if (bufferLevel > 0) {
        for (int i = 0; i < bufferLevel; i++) {
          serialBuffer[i][0] = ID_MULTI_PACKET;
        }
      }
      serialBuffer[bufferLevel][0] = ID_MULTI_PACKET_STOP;
      timeLastPacketSent = millis();  // prime the timer to send verbose packets
      bufferLevelCounter = 0;
      serialBytesToSend = true;
    }

    void OpenBCI_Ganglion::loadNewLine() {
      serialBuffer[bufferLevel][serialIndex[bufferLevel]] = '\n';
      serialIndex[bufferLevel]++;           // count up the buffer size
      if (serialIndex[bufferLevel] == SERIAL_BUFFER_LENGTH) {  // when the buffer is full,
        bufferLevel++;        // next buffer please
      }
    }



    void OpenBCI_Ganglion::loadString(char* thatString, int numChars, boolean addNewLine) {
      for (int i = 0; i < numChars; i++) {
        serialBuffer[bufferLevel][serialIndex[bufferLevel]] = thatString[i];
        serialIndex[bufferLevel]++;           // count up the buffer size
        if (serialIndex[bufferLevel] == SERIAL_BUFFER_LENGTH) { // when the buffer is full,
          bufferLevel++;        // next buffer please
        }
      }
      if (addNewLine) {
        loadNewLine();
      }
    }

    void OpenBCI_Ganglion::loadChar(char thatChar, boolean addNewLine) {
      serialBuffer[bufferLevel][serialIndex[bufferLevel]] = thatChar;
      serialIndex[bufferLevel]++;           // count up the buffer size
      if (serialIndex[bufferLevel] == SERIAL_BUFFER_LENGTH) {  // when the buffer is full,
        bufferLevel++;        // next buffer please
      }
      if (addNewLine) {
        loadNewLine();
      }
    }

    void OpenBCI_Ganglion::loadHex(int hexBytes, int numBytes, boolean addNewLine) {
      byte nibble;
      int numBits = (numBytes * 8) - 4;
      loadString("0x", 2, false);
      for (int i = numBits; i >= 0; i -= 4) {
        nibble = ((hexBytes >> i) & 0x0F) + '0';
        if (nibble > '9') {
          nibble += 7;
        }
        serialBuffer[bufferLevel][serialIndex[bufferLevel]] = nibble;
        serialIndex[bufferLevel]++;           // count up the buffer size
        if (serialIndex[bufferLevel] == SERIAL_BUFFER_LENGTH) { // when the buffer is full,
          bufferLevel++;        // next buffer please
        }
      }
      if (addNewLine) {
        loadNewLine();
      }
    }


    void OpenBCI_Ganglion::initSerialBuffer() {            // initialize 2D serial buffer in normal mode
      for (int i = 0; i < SERIAL_BUFFER_LENGTH; i++) {
        serialIndex[i] = 1;        // save byte 0 for the byte ID
      }
    }


    void OpenBCI_Ganglion::loadInt(int i, boolean addNewLine) {
      int integer = i;
      int digitCounter = 0;
      char digit[10];

      while (integer > 0) {
        digit[digitCounter] = (integer % 10) + '0';
        integer /= 10;
        digitCounter++;
      }

      for (int i = digitCounter - 1; i >= 0; i--) {
        serialBuffer[bufferLevel][serialIndex[bufferLevel]] = digit[i];
        serialIndex[bufferLevel]++;           // count up the buffer size
        if (serialIndex[bufferLevel] == SERIAL_BUFFER_LENGTH) { // when the buffer is full,
          bufferLevel++;        // next buffer please
        }
      }
      if (addNewLine) {
        loadNewLine();
      }
    }



    // DECODE THE RECEIVED COMMAND CHARACTER
    void OpenBCI_Ganglion::parseChar(char token) {
      switch (token) {
        // TURN OFF CHANNELS
        case DEACTIVATE_CHANNEL_1:
          changeChannelState_maintainRunningState(1, DEACTIVATE); break;
          // Serial.println("Deactivate 1"); break;
        case DEACTIVATE_CHANNEL_2:
          changeChannelState_maintainRunningState(2, DEACTIVATE); break;
          // Serial.println("Deactivate 2"); break;
        case DEACTIVATE_CHANNEL_3:
          changeChannelState_maintainRunningState(3, DEACTIVATE); break;
          // Serial.println("Deactivate 3"); break;
        case DEACTIVATE_CHANNEL_4:
          changeChannelState_maintainRunningState(4, DEACTIVATE); break;
          // Serial.println("Deactivate 4"); break;
        // TURN ON CHANNELS
        case ACTIVATE_CHANNEL_1:
          changeChannelState_maintainRunningState(1, ACTIVATE); break;
          // Serial.println("Activate 1"); break;
        case ACTIVATE_CHANNEL_2:
          changeChannelState_maintainRunningState(2, ACTIVATE); break;
          // Serial.println("Activate 2"); break;
        case ACTIVATE_CHANNEL_3:
          changeChannelState_maintainRunningState(3, ACTIVATE); break;
          // Serial.println("Activate 3"); break;
        case ACTIVATE_CHANNEL_4:
          changeChannelState_maintainRunningState(4, ACTIVATE); break;
          // Serial.println("Activate 4"); break;

        case START_DATA_STREAM:
          if (!BLEconnected) {
            loadString("BLE not connected: abort startRunning",37,true);
            prepToSendBytes();
          } else if (!is_running){
            if(testingImpedance){ endImpedanceTest(); }
            requestToStartRunning = true;
            startRunning();  // returns value of is_running = true
          }
          break;
        case ENABLE_SYNTHETIC_DATA:
          if (!is_running) {
            loadString("enable square wave", 18, true);
            prepToSendBytes();
          }
          streamSynthetic = true;
          break;
        case DISABLE_SYNTHETIC_DATA:
          if (!is_running) {
            loadString("disable square wave", 19, true);
            prepToSendBytes();
          }
          streamSynthetic = false;
          break;
        case STOP_DATA_STREAM:
          stopRunning();    // returns value of is_running = false
          loadString("stop running",12,true);
          prepToSendBytes();
          break;
        case ENABLE_ACCELEROMETER:
          useAccel = true;
          if (!is_running) {
            loadString("accelerometer enabled", 21, true);
            prepToSendBytes();
          } else {
            accelOnEdge = true;
          }
          enable_LIS2DH();
          break;
        case DISABLE_ACCELEROMETER:
          useAccel = false;
          if (!is_running) {
            loadString("accelerometer disabled", 22, true);
            prepToSendBytes();
          } else {
            accelOffEdge = true;
          }
          disable_LIS2DH();
          break;
        case SOFT_RESET:  // CONFIG
          if(is_running){ stopRunning(); }
          startFromScratch(gain, sps);
          break;
        case REPORT_REGISTER_SETTINGS:  // PRINT ALL REGISTER VALUES
          if(!is_running){ printAllRegisters_Serial(); }
          break;
        case OPENBCI_Z_TEST_START:   // 'z'
          if (is_running) {
            wasRunningWhenCalled = true;
            stopRunning();
            // Serial.println("stopRunning");
          } else {
            wasRunningWhenCalled = false;
          }
          testingImpedance = true;
          ACwaveTest = false;
          channelUnderZtest = 1;
          break;
        case OPENBCI_Z_TEST_STOP:   // 'Z'
          // Serial.println("received Z");
          endImpedanceTest();
          if (wasRunningWhenCalled) {
            startRunning();
          }
          break;
        case ENABLE_OTA:  // '>'
          requestForOTAenable = true;
          if(!BLEconnected){ clearForOTA = true; }
          break;
        default:
          if(!BLEconnected){
            loadString("parseChar got: ", 17, false); loadChar(token, true);
            prepToSendBytes();
          }
          break;
      }
    }


    // <<<<<<<<<<<<<<<<<<<<<<<<<  END OF COM FUNCTIONS >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    // *************************************************************************************
    // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<< SIMBLEE FUNCTIONS  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    void SimbleeBLE_onConnect()
    {
      ganglion.channelMask = 0x00000000;
      ganglion.BLEconnected = true;
      digitalWrite(LED,HIGH);
    }



    void SimbleeBLE_onDisconnect()
    {
      ganglion.BLEconnected = false;
      if(!ganglion.writingToSD){
        ganglion.stopRunning();
        ganglion.endImpedanceTest();
        ganglion.useAccel = false;
        ganglion.useAux = false;
        ganglion.LED_timer = millis();
      }
      if(ganglion.requestForOTAenable){
        ganglion.requestForOTAenable = false;
        ganglion.clearForOTA = true;
      }

    }

    void SimbleeBLE_onReceive(char *data, int len)
    {
      ganglion.BLEchar = data[0];
      ganglion.gotBLE = true;
    }


OpenBCI_Ganglion ganglion;
