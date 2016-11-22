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
// > enableHeartbeat/bool enabled
#define ENABLE_HEARTBEAT 5
// > enablePin/address/type/b16 rate
#define ENABLE_PIN 6
// > heartbeat
#define HEARTBEAT 7
// > setDebug/bool enabled
#define SET_DEBUG 8
// > setSerialRate/b32 rate
#define SET_SERIAL_RATE 9
// > softReset/
#define SOFT_RESET 10
// > echo/bu32 sInt
#define ECHO 11
// < publishEcho/bu32 sInt
#define PUBLISH_ECHO 12
// > controllerAttach/serialPort
#define CONTROLLER_ATTACH 13
// > customMsg/[] msg
#define CUSTOM_MSG 14
// < publishCustomMsg/[] msg
#define PUBLISH_CUSTOM_MSG 15
// > deviceDetach/deviceId
#define DEVICE_DETACH 16
// > i2cAttach/deviceId/i2cBus/deviceType/deviceAddress
#define I2C_ATTACH 17
// > i2cRead/deviceId/deviceAddress/size
#define I2C_READ 18
// > i2cWrite/deviceId/deviceAddress/[] data
#define I2C_WRITE 19
// > i2cWriteRead/deviceId/deviceAddress/readSize/writeValue
#define I2C_WRITE_READ 20
// > neoPixelAttach/deviceId/pin/b32 numPixels
#define NEO_PIXEL_ATTACH 21
// > neoPixelSetAnimation/deviceId/animation/red/green/blue/b16 speed
#define NEO_PIXEL_SET_ANIMATION 22
// > neoPixelWriteMatrix/deviceId/[] buffer
#define NEO_PIXEL_WRITE_MATRIX 23
// > analogWrite/pin/value
#define ANALOG_WRITE 24
// > digitalWrite/pin/value
#define DIGITAL_WRITE 25
// > disablePin/pin
#define DISABLE_PIN 26
// > disablePins
#define DISABLE_PINS 27
// > pinMode/pin/mode
#define PIN_MODE 28
// < publishAttachedDevice/deviceId/str deviceName
#define PUBLISH_ATTACHED_DEVICE 29
// < publishBoardStatus/b16 microsPerLoop/b16 sram/[] deviceSummary
#define PUBLISH_BOARD_STATUS 30
// < publishDebug/str debugMsg
#define PUBLISH_DEBUG 31
// < publishMessageAck/function
#define PUBLISH_MESSAGE_ACK 32
// < publishSensorData/deviceId/[] data
#define PUBLISH_SENSOR_DATA 33
// < publishServoEvent/deviceId/eventType/currentPos/targetPos
#define PUBLISH_SERVO_EVENT 34
// > setTrigger/pin/triggerValue
#define SET_TRIGGER 35
// > setDebounce/pin/delay
#define SET_DEBOUNCE 36
// > serialRelay/deviceId/serialPort/[] relayData
#define SERIAL_RELAY 37
// > servoAttach/deviceId/pin/initPos/b16 initVelocity
#define SERVO_ATTACH 38
// > servoEnablePwm/deviceId/pin
#define SERVO_ENABLE_PWM 39
// > servoDisablePwm/deviceId
#define SERVO_DISABLE_PWM 40
// > servoSetMaxVelocity/deviceId/b16 maxVelocity
#define SERVO_SET_MAX_VELOCITY 41
// > servoSetVelocity/deviceId/b16 velocity
#define SERVO_SET_VELOCITY 42
// > servoSweepStart/deviceId/min/max/step
#define SERVO_SWEEP_START 43
// > servoSweepStop/deviceId
#define SERVO_SWEEP_STOP 44
// > servoWrite/deviceId/target
#define SERVO_WRITE 45
// > servoWriteMicroseconds/deviceId/b16 ms
#define SERVO_WRITE_MICROSECONDS 46



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
