/*
   insert header here

*/
#ifndef _____OpenBCI_Ganglion_Library__
#define _____OpenBCI_Ganglion_Library__


#include <SPI.h>
#include <SimbleeBLE.h>
#include <OpenBCI_Wifi_Master.h>
#include <OpenBCI_Wifi_Master_Definitions.h>
#include "Definitions_Ganglion.h"

class OpenBCI_Ganglion {
public:


  typedef enum PACKET_TYPE {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_RAW_AUX,
    PACKET_TYPE_USER_DEFINED,
    PACKET_TYPE_ACCEL_TIME_SET,
    PACKET_TYPE_ACCEL_TIME_SYNC,
    PACKET_TYPE_RAW_AUX_TIME_SET,
    PACKET_TYPE_RAW_AUX_TIME_SYNC,
    PACKET_TYPE_IMPEDANCE
  };

  typedef enum SAMPLE_RATE {
    SAMPLE_RATE_25600,
    SAMPLE_RATE_12800,
    SAMPLE_RATE_6400,
    SAMPLE_RATE_3200,
    SAMPLE_RATE_1600,
    SAMPLE_RATE_800,
    SAMPLE_RATE_400,
    SAMPLE_RATE_200
  };

  OpenBCI_Ganglion();

  SAMPLE_RATE curSampleRate;

  void    processIncomingSampleRate(char);
  void    setSampleRate(uint8_t);
  const char * getSampleRate(void);

  void initialize(void);
  void makeUniqueId(void);
  void blinkLED(void);
  void startFromScratch(unsigned long);
  void printAllRegisters_Serial();
  void processData(void);
  boolean startRunning(void);
  boolean stopRunning(void);
  // void sendChannelData(void);
  int changeChannelState_maintainRunningState(int, int);
  void initSyntheticData(void);
  void startRunningSynthetic(void);
  void incrementSyntheticChannelData(void);
  void buildRawPacket(void);
  void sendRawPacket(void);
  void compressData18(void);
  void compressData19(void);
  void sendCompressedPacket18(void);
  void sendCompressedPacket19(void);
  void resendPacket(byte);  // this trip to the past will happen in the future
  void testImpedance(void);
  void endImpedanceTest(void);
  double convertRawGanglionImpedanceToTarget(double);
  void updateDAC(word);
  void updateDAC(void);
  void zeroDAC(void);
  float get_Zvalue(int);
  word getDACzeroPosition();
  void logData_Serial(void);
  void readShuntSensor(void);
  void gotoTarget(float, float );
  void rampTest(void);
  void changeZtestForChannel(int, int);
  void updateAccelerometerData(void);
  void config_LIS2DH(void);
  void enable_LIS2DH(void);
  void disable_LIS2DH(void);
  word LIS2DH_readTemp(void);
  byte LIS2DH_read(byte);
  void LIS2DH_write(byte, byte);
  short LIS2DH_read16(byte);
  float getG(byte);
  void LIS2DH_readAllRegs_Serial();
  // int MCP_ISR(uint32_t);
  void config_MCP3912(unsigned long);
  void updateMCPdata(void);
  void MCP_sendCommand(byte, byte);
  long MCP_readRegister(void);
  void MCP_writeRegister(unsigned long);
  void MCP_turnOnChannels(void);
  void MCP_turnOffAllChannels(void);
  void MCP_readAllRegs_Serial(void);
  void MCP_printRegisterName_Serial(byte);
  void MCP_readAllRegs(void);
  void MCP_printRegisterName(byte);
  boolean eventSerial(void);
  void sendSerialBytesBlocking(void);
  void prepToSendBytes(void);
  void loadNewLine(void);
  void loadString(const char*, int, boolean);
  void loadString(const char*);
  void loadString(void);
  void loadlnString(const char*);
  void loadlnString(void);
  void loadChar(char, boolean);
  void loadHex(int, int, boolean);
  void initSerialBuffer(void);
  void loadInt(int i, boolean);
  void parseChar(char);
  void parseCharWifi(char);
  void printFailure(void);
  void printSampleRate(void);
  void printSuccess(void);
  uint8_t * getGains(void);

  boolean settingSampleRate = false;

  uint8_t advdata[15] =
  {
    14,    // length // 0
    0x09,  // complete local name type // 1
    0x47,  // 'G' // 2
    0x61,  // 'a' // 3
    0x6E,  // 'n' // 4
    0x67,  // 'g' // 5
    0x6C,  // 'l' // 6
    0x69,  // 'i' // 7
    0x6F,  // 'o' // 8
    0x6E,  // 'n' // 9
    0x2D,  // '-' // 10
    0x54,  // 'T' // 11
    0x41,  // 'A' // 12
    0x43,  // 'C' // 13
    0x4f,  // 'O' // 14
  };

  // int LED_delayTime = 300;
  unsigned int LED_timer = 0;
  boolean LED_state = true;
  boolean is_running = false;
  boolean streamSynthetic = false;
  boolean serialBytesToSend = false;
  char serialBuffer[SERIAL_BUFFER_LENGTH][SERIAL_BUFFER_LENGTH];
  int serialIndex[SERIAL_BUFFER_LENGTH];
  unsigned long timeLastPacketSent; // used to time sending verbose BLE packets
  int bufferLevel;
  int bufferLevelCounter;
  boolean wasRunningWhenCalled = false;
  boolean useAux = false;
  boolean newAuxData = false;
  char auxData[3];
  char inChar;
  boolean gotSerial = false;
  boolean commandFromSPI = false;

  //  >>>>  LIS2DH STUFF  <<<<

  boolean useAccel = false;
  int8_t axisData[3]; // holds accelerometer x,y,z
  boolean accelOnEdge = false;
  boolean accelOffEdge = false;
  byte ID;  // holds LIS2DH device ID
  float scale_factor_gs_per_count = 0.016; // assume +/-4g, normal mode. 8bits left justified
  volatile boolean LIS_dataReady = false;
  boolean newAccelData = false;


  //  >>>>  MCP3912 STUFF <<<<
  byte compression_ring[RING_SIZE][MAX_BYTES_PER_PACKET];
  int ringBufferLevel = 0;    // const int compressionMask = 0xFFFFFFF8;
  int channelData[4];            // holds channel data
  int lastChannelData[4];       // holds last channel data
  byte rawChannelData[24];
  volatile byte sampleCounter = 0xFF;    // sample counter, for real
  volatile boolean MCP_dataReady = false;
  volatile boolean zeroth = false;
  volatile boolean first = false;


  unsigned int sampleNumber;
  unsigned long channelMask = 0x00000000;  // used to turn on selected channels
  unsigned long channelEnable[4] = {ENABLE_0, ENABLE_1, ENABLE_2, ENABLE_3};
  unsigned long channelDisable[4] = {DISABLE_0, DISABLE_1, DISABLE_2, DISABLE_3};
  byte channelAddress[4] = {CHAN_0,CHAN_1,CHAN_2,CHAN_3};
  unsigned long gain = GAIN_1;
  // unsigned long sps = SAMPLE_200;
  boolean requestToStartRunning = false;
  unsigned long thisTime;
  unsigned long thatTime;
  unsigned long thisStampTime;
  unsigned long thatStampTime;
  int timeDifference;
  boolean rising[4] = {true,true,true,true};
  unsigned long regVal;


  //  >>>>  IMPEDANCE TESTING STUFF <<<<

  boolean testingImpedance = false;
  int channelUnderZtest;
  float uAmp_Value = 0.0;           // value of measured current
  float ADC_volts_per_count = 3.0/1023.0;    //0.00293255  // 3/((2^10)-1)
  // float DAC_uVolts_per_count = 3.0/4095.0;  // .0007236;  // (3/((2^12)-1))
  float DAC_volts_per_count = 3.0/4095.0;    //0.0007236;  // 3/((2^12)-1)
  float noise = 0.1;
  short DAC_position;   // 12bit DAC resolution (4096) ~0.8mV DAC, ~5nA tXCS
  float DAC_voltage;    // DAC position converted to volts
  short DACmidline = 2047;
  int Ohms;
  short DACmask;   // used in update to add control bits
  boolean increment = true;
  boolean impedanceTesting = false;
  int impedanceSwitch[5] = {Z_TEST_1, Z_TEST_2, Z_TEST_3, Z_TEST_4, Z_TEST_REF};
  int currentChannelSetting;
  int leadOffSetting;
  int currentChannelZeroPosition;
  int currentChannelPlusTenPosition;
  int currentChannelMinusTenPosition;
  boolean rampTesting = false;
  boolean Z_noiseTesting = false;
  boolean plusTen = false;
  boolean minusTen = false;
  boolean zero = false;
  unsigned long testTimer;
  int testTime = 10000;          // 10 second test time to collect data
  unsigned long sampleTimer;
  int rampSampleTime = 100;      // 100 millisecond time between samples (10Hz)
  int noiseSampleTime = 10000;   // 10000 microsecond time between samples (100Hz)
  int gotoSampleTime = 2000;     //
  int numCurrentMeasurements = 4;
  int currentMeasurementCounter;

  //  AC WAVEFORM STUPH
  boolean ACwaveTest = false;
  unsigned long halfPeriodTimer;    // used to time each cycle of the AC wave

  word realZeroPosition = 2048;     // have to discover the '0Amps' point later
  boolean ACrising = true;            // start the square wave going up
  int Z_testTime = 1000;            // test for a second?
  unsigned long Z_testTimer;        //

  unsigned long uAsampleTimer;
  int halfWave = 100;
  int uAsampleCounter;
  int currentCounts;
  int maxPosCurrentCounts;
  int minNegCurrentCounts;
  int currentCountsAtZeroAmps;
  int peakCurrentCounts;
  int positiveRunningTotal;
  int negativeRunningTotal;
  int positiveSampleCounter;
  int negativeSampleCounter;
  boolean edge = false;
  int increased, decreased, steady;
  int negativeMean, positiveMean;
  int positiveEdge, negativeEdge, negativeEdgeCounter, positiveEdgeCounter;

  //  <<<< BLE STUFF >>>>
  char radioBuffer[20];
  char resendBuffer[20];
  boolean BLEconnected = false;
  char BLEchar[20];
  int BLEcharHead = 0;
  int BLEcharTail = 0;
  unsigned long rssiTimer;
  boolean requestForOTAenable = false;
  boolean clearForOTA = false;
  boolean writingToSD = false;
  boolean useSerial = false; // used for testing stuph
  int syntheticFrequency = 500;

};

extern OpenBCI_Ganglion ganglion;

#endif
