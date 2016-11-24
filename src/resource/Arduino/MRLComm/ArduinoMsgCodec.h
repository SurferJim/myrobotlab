#ifndef ArduinoMsgCodec_h
#define ArduinoMsgCodec_h

/*******************************************************************
 * MRLCOMM FUNCTION GENERATED INTERFACE
 * these defines are generated with :
 *							arduinoMsgs.schema
 * 							ArduinoMsgGenerator
 * 							src\resource\Arduino\generate\ArduinoMsgCodec.template.h
 */

/*******************************************************************
 * serial protocol functions
 */
#define MAGIC_NUMBER            170 // 10101010
#define MAX_MSG_SIZE			64


// < publishMRLCommError/str errorMsg
#define PUBLISH_MRLCOMM_ERROR 1
// > getBoardInfo
#define GET_BOARD_INFO 2
// < publishBoardInfo/version/boardType
#define PUBLISH_BOARD_INFO 3
// > enableBoardStatus/bool enabled
#define ENABLE_BOARD_STATUS 4
// > enablePin/address/type/b16 rate
#define ENABLE_PIN 5
// > setDebug/bool enabled
#define SET_DEBUG 6
// > setSerialRate/b32 rate
#define SET_SERIAL_RATE 7
// > softReset
#define SOFT_RESET 8
// > enableAck/bool enabled
#define ENABLE_ACK 9
// < publishAck/function
#define PUBLISH_ACK 10
// > enableHeartbeat/bool enabled
#define ENABLE_HEARTBEAT 11
// > heartbeat
#define HEARTBEAT 12
// < publishHeartbeat
#define PUBLISH_HEARTBEAT 13
// > echo/bu32 sInt
#define ECHO 14
// < publishEcho/bu32 sInt
#define PUBLISH_ECHO 15
// > controllerAttach/serialPort
#define CONTROLLER_ATTACH 16
// > customMsg/[] msg
#define CUSTOM_MSG 17
// < publishCustomMsg/[] msg
#define PUBLISH_CUSTOM_MSG 18
// > deviceDetach/deviceId
#define DEVICE_DETACH 19
// > i2cAttach/deviceId/i2cBus/deviceType/deviceAddress
#define I2C_ATTACH 20
// > i2cRead/deviceId/deviceAddress/size
#define I2C_READ 21
// > i2cWrite/deviceId/deviceAddress/[] data
#define I2C_WRITE 22
// > i2cWriteRead/deviceId/deviceAddress/readSize/writeValue
#define I2C_WRITE_READ 23
// > neoPixelAttach/deviceId/pin/b32 numPixels
#define NEO_PIXEL_ATTACH 24
// > neoPixelSetAnimation/deviceId/animation/red/green/blue/b16 speed
#define NEO_PIXEL_SET_ANIMATION 25
// > neoPixelWriteMatrix/deviceId/[] buffer
#define NEO_PIXEL_WRITE_MATRIX 26
// > analogWrite/pin/value
#define ANALOG_WRITE 27
// > digitalWrite/pin/value
#define DIGITAL_WRITE 28
// > disablePin/pin
#define DISABLE_PIN 29
// > disablePins
#define DISABLE_PINS 30
// > pinMode/pin/mode
#define PIN_MODE 31
// < publishAttachedDevice/deviceId/str deviceName
#define PUBLISH_ATTACHED_DEVICE 32
// < publishBoardStatus/b16 microsPerLoop/b16 sram/[] deviceSummary
#define PUBLISH_BOARD_STATUS 33
// < publishDebug/str debugMsg
#define PUBLISH_DEBUG 34
// < publishPinArray/[] data
#define PUBLISH_PIN_ARRAY 35
// > setTrigger/pin/triggerValue
#define SET_TRIGGER 36
// > setDebounce/pin/delay
#define SET_DEBOUNCE 37
// > serialRelay/deviceId/serialPort/[] relayData
#define SERIAL_RELAY 38
// > servoAttach/deviceId/pin/initPos/b16 initVelocity
#define SERVO_ATTACH 39
// > servoEnablePwm/deviceId/pin
#define SERVO_ENABLE_PWM 40
// > servoDisablePwm/deviceId
#define SERVO_DISABLE_PWM 41
// > servoSetMaxVelocity/deviceId/b16 maxVelocity
#define SERVO_SET_MAX_VELOCITY 42
// > servoSetVelocity/deviceId/b16 velocity
#define SERVO_SET_VELOCITY 43
// > servoSweepStart/deviceId/min/max/step
#define SERVO_SWEEP_START 44
// > servoSweepStop/deviceId
#define SERVO_SWEEP_STOP 45
// > servoWrite/deviceId/target
#define SERVO_WRITE 46
// > servoWriteMicroseconds/deviceId/b16 ms
#define SERVO_WRITE_MICROSECONDS 47



/*******************************************************************
 * BOARD TYPE
 */
#define BOARD_TYPE_ID_UNKNOWN 0
#define BOARD_TYPE_ID_MEGA    1
#define BOARD_TYPE_ID_UNO     2

#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_ADK)
  #define BOARD BOARD_TYPE_ID_MEGA
#elif defined(ARDUINO_AVR_UNO)
  #define BOARD BOARD_TYPE_ID_UNO
#else
  #define BOARD BOARD_TYPE_ID_UNKNOWN
#endif

#endif
