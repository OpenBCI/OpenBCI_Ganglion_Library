//
//

#ifndef _Definitions_Ganglion_h
#define _Definitions_Ganglion_h

#define LED 23
#define LED_DELAY_TIME 300
#define RING_SIZE 101
#define SERIAL_BUFFER_LENGTH  20
#define MAX_BYTES_PER_PACKET  19

// Byte IDs
#define ID_RAW_DATA 0
#define ID_SAMPLE_18_MIN 1
#define ID_SAMPLE_18_MAX 100
#define ID_SAMPLE_19_MIN 101
#define ID_SAMPLE_19_MAX 200
#define ID_Z_1  201
#define ID_Z_2  202
#define ID_Z_3  203
#define ID_Z_4  204
#define ID_Z_R  205
#define ID_MULTI_PACKET 206
#define ID_MULTI_PACKET_STOP  207

// Serial Command Characters
#define DEACTIVATE_CHANNEL_1 '1'
#define DEACTIVATE_CHANNEL_2 '2'
#define DEACTIVATE_CHANNEL_3 '3'
#define DEACTIVATE_CHANNEL_4 '4'
#define ACTIVATE_CHANNEL_1 '!'
#define ACTIVATE_CHANNEL_2 '@'
#define ACTIVATE_CHANNEL_3 '#'
#define ACTIVATE_CHANNEL_4 '$'
#define START_DATA_STREAM 'b'
#define STOP_DATA_STREAM 's'
#define ENABLE_ACCELEROMETER 'n'
#define DISABLE_ACCELEROMETER 'N'
#define ENABLE_SYNTHETIC_DATA '['
#define DISABLE_SYNTHETIC_DATA ']'
#define SOFT_RESET 'v'
#define REPORT_REGISTER_SETTINGS '?'
#define ENABLE_OTA '>'
#define OPENBCI_SAMPLE_RATE_SET '~'

#define OPENBCI_CHANNEL_1 '1'
#define OPENBCI_CHANNEL_2 '2'
#define OPENBCI_CHANNEL_3 '3'
#define OPENBCI_CHANNEL_4 '4'
#define OPENBCI_CHANNEL_REF '5'

#define OPENBCI_Z_TEST_START 'z'
#define OPENBCI_Z_TEST_STOP 'Z'

//  MCP
#define DEACTIVATE 0
#define ACTIVATE 1
#define ENABLE_0   0x000E0000
#define ENABLE_1   0x000D0000
#define ENABLE_2   0x000B0000
#define ENABLE_3   0x00070000
#define DISABLE_0  0x00010000
#define DISABLE_1  0x00020000
#define DISABLE_2  0x00040000
#define DISABLE_3  0x00080000

#define MCP_SS  7   // MCP Slave Select on Simblee pin 13
#define MCP_DRDY 8   // MCP DataReady on Simblee pin 8
#define MCP_RST 20   // MCP Reset: active low

#define MCP_ADD  0x40
#define MCP_READ  0x01
#define MCP_WRITE  0x00

#define DEV_ADD  0x40

#define CHAN_0  0x00       //00   |        | -GROUP
#define CHAN_1  0x02       //01   |-TYPE  _|
#define CHAN_2  0x04       //02   |        | -GROUP
#define CHAN_3  0x06       //03  _|       _|

#define MOD_VAL  0x10      //08   |        |
#define PHASE  0x14        //0A   |        |-GROUP
#define GAIN  0x16         //0B   |       _|
#define STATUSCOM  0x18    //0C   |        |
#define CONFIG_0  0x1A     //0D   |        |-GROUP
#define CONFIG_1  0x1C     //0E   |       _|
#define OFFCAL_0  0x1E     //0F   |-TYPE   | -GROUP
#define GAINCAL_0  0x20    //10   |       _|
#define OFFCAL_1  0x22     //11   |        | -GROUP
#define GAINCAL_1  0x24    //12   |       _|
#define OFFCAL_2  0x26     //13   |        | -GROUP
#define GAINCAL_2  0x28    //14   |       _|
#define OFFCAL_3  0x2A     //15   |        | -GROUP
#define GAINCAL_3  0x2C    //16   |       _|
#define LOK_CRC  0x3E      //1F  _|        >-GROUP

#define GAIN_1  0x00000000
#define GAIN_2  0x00000249
#define GAIN_4  0x00000492
#define GAIN_8  0x000006CB
#define GAIN_16 0x00000924
#define GAIN_32 0x00000B6D

//  0x0038E050
// #define SAMPLE_25   0x00030000
// #define SAMPLE_50   0x00020000
// #define SAMPLE_100  0x00010000
// #define SAMPLE_200  0x00000000


//#define OPENBCI_NUMBER_OF_BYTES_SETTINGS_Z_TEST 3
#define HALF_PERIOD 3000
#define HALF_WAVE 100
#define SHUNT 100
#define SHUNT_SENSOR_GAIN 1000
#define UA_SAMPLE_TIME 500
#define UA_SAMPLE_LIMIT 1000


/*
 *  AD5621
 */
#define DAC_MASK  0x3FFC
#define SHUNT_SENSOR 6     // reads the current shunt sensor
#define DAC_SS    19       // DAC slave select
#define Z_TEST_1  30       // pin control for Z test channel 1
#define Z_TEST_2  29       // pin control for Z test channel 2
#define Z_TEST_3  28       // pin control for Z test channel 3
#define Z_TEST_4  25       // pin control for Z test channel 4
#define Z_TEST_REF  22     // pin control for Z test channel REF
// CONTROL BYTES
#define DAC_EN    0x0000  // enables setting the DAC output 12 bit resolution
#define DAC_1K    0x7000  // disables DAC with output tied 1K to GND
#define DAC_100K  0x8000  // disables DAC with output tied 100K to GND
#define DAC_Z     0xC000  // disables DAC with output in high Z
#define DAC_MAX   0xFFFF  // highest possible value of DAC = 4095




/*
 * LIS2DH
 */

#define LIS2DH_SS   10
#define LIS_DRDY    13

#define READ_REG    0x80
#define READ_MULTI    0x40

#define LIS3DH_MODE   3 // c pol =1, c pha = 1, mode = 3

#define STATUS_REG_AUX  0x07
#define OUT_TEMP_L      0x0C
#define OUT_TEMP_H      0x0D
#define INT_COUNTER_REG 0x0E
#define WHO_AM_I        0x0F
#define TEMP_CFG_REG    0x1F
#define CTRL_REG1       0x20
#define CTRL_REG2       0x21
#define CTRL_REG3       0x22
#define CTRL_REG4       0x23
#define CTRL_REG5       0x24
#define CTRL_REG6       0x25
#define REFERENCE       0x26
#define STATUS_REG2     0x27
#define OUT_X_L         0x28
#define OUT_X_H         0x29
#define OUT_Y_L         0x2A
#define OUT_Y_H         0x2B
#define OUT_Z_L         0x2C
#define OUT_Z_H         0x2D
#define FIFO_CTRL_REG   0x2E
#define FIFO_SRC_REG    0x2F
#define INT1_CFG        0x30
#define INT1_SOURCE     0x31
#define INT1_THS        0x32
#define INT1_DURATION   0x33
#define INT2_CFG        0x34
#define INT2_SOURCE     0x35
#define INT2_THS        0x36
#define INT2_DURATION   0x37
#define CLICK_CFG       0x38
#define CLICK_SRC       0x39
#define CLICK_THS       0x3A
#define TIME_LIMIT      0x3B
#define TIME_LATENCY    0x3C
#define TIME_WINDOW     0x3D
#define ACT_THS         0x3E
#define ACT_DUR         0x3F

/*
* SD CARD
*/
#define SD_SS 17

/** Wifi Stuff */
#define NUM_CHANNELS 4
#define OPENBCI_WIFI_ATTACH '{'
#define OPENBCI_WIFI_REMOVE '}'
#define OPENBCI_WIFI_STATUS ':'
#define OPENBCI_WIFI_RESET ';'

#endif
