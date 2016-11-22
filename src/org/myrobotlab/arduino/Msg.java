package org.myrobotlab.arduino;

import org.myrobotlab.logging.Level;

/**
 * <pre>
 * 
 Welcome to Msg.java
 Its created by running ArduinoMsgGenerator
 which combines the MrlComm message schema (src/resource/Arduino/arduinoMsg.schema)
 with the cpp template (src/resource/Arduino/generate/Msg.template.java)

 	Schema Type Conversions

	Schema      ARDUINO					Java							Range
	none		byte/unsigned char		int (cuz Java byte bites)		1 byte - 0 to 255
	boolean		boolean					boolean							0 1
    b16			int						int (short)						2 bytes	-32,768 to 32,767
    b32			long					int								4 bytes -2,147,483,648 to 2,147,483, 647
    bu32		unsigned long			long							0 to 4,294,967,295
    str			char*, size				String							variable length
    []			byte[], size			int[]							variable length

 All message editing should be done in the arduinoMsg.schema

 The binary wire format of an Arduino is:

 MAGIC_NUMBER|MSG_SIZE|METHOD_NUMBER|PARAM0|PARAM1 ...
 
 </pre>

 */

import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.logging.LoggingFactory;
import org.myrobotlab.service.VirtualArduino;
import java.util.Arrays;
import org.myrobotlab.service.Arduino;
import org.myrobotlab.service.Runtime;
import org.myrobotlab.service.Servo;
import org.myrobotlab.service.interfaces.SerialDevice;
import org.slf4j.Logger;

/**
 * Singlton messaging interface to an Arduino
 *
 * @author GroG
 *
 */

public class Msg {

	public static final int MAX_MSG_SIZE = 64;
	public static final int MAGIC_NUMBER = 170; // 10101010
	public static final int MRLCOMM_VERSION = 41;

	// ------ device type mapping constants

	public static final int DEVICE_TYPE_NOT_FOUND = 0;

	public static final int DEVICE_TYPE_ARDUINO = 1;
	public static final int DEVICE_TYPE_ULTRASONIC = 4;
	public static final int DEVICE_TYPE_STEPPER = 5;
	public static final int DEVICE_TYPE_MOTOR = 6;
	public static final int DEVICE_TYPE_SERVO = 7;
	public static final int DEVICE_TYPE_I2C = 8;
	public static final int DEVICE_TYPE_NEOPIXEL = 9;

	// < publishMRLCommError/str errorMsg
	public final static int PUBLISH_MRLCOMM_ERROR = 1;
	// > getBoardInfo
	public final static int GET_BOARD_INFO = 2;
	// < publishBoardInfo/version/boardType
	public final static int PUBLISH_BOARD_INFO = 3;
	// > enableBoardStatus/bool enabled
	public final static int ENABLE_BOARD_STATUS = 4;
	// > enableHeartbeat/bool enabled
	public final static int ENABLE_HEARTBEAT = 5;
	// > enablePin/address/type/b16 rate
	public final static int ENABLE_PIN = 6;
	// > heartbeat
	public final static int HEARTBEAT = 7;
	// > setDebug/bool enabled
	public final static int SET_DEBUG = 8;
	// > setSerialRate/b32 rate
	public final static int SET_SERIAL_RATE = 9;
	// > softReset/
	public final static int SOFT_RESET = 10;
	// > echo/bu32 sInt
	public final static int ECHO = 11;
	// < publishEcho/bu32 sInt
	public final static int PUBLISH_ECHO = 12;
	// > controllerAttach/serialPort
	public final static int CONTROLLER_ATTACH = 13;
	// > customMsg/[] msg
	public final static int CUSTOM_MSG = 14;
	// < publishCustomMsg/[] msg
	public final static int PUBLISH_CUSTOM_MSG = 15;
	// > deviceDetach/deviceId
	public final static int DEVICE_DETACH = 16;
	// > i2cAttach/deviceId/i2cBus/deviceType/deviceAddress
	public final static int I2C_ATTACH = 17;
	// > i2cRead/deviceId/deviceAddress/size
	public final static int I2C_READ = 18;
	// > i2cWrite/deviceId/deviceAddress/[] data
	public final static int I2C_WRITE = 19;
	// > i2cWriteRead/deviceId/deviceAddress/readSize/writeValue
	public final static int I2C_WRITE_READ = 20;
	// > neoPixelAttach/deviceId/pin/b32 numPixels
	public final static int NEO_PIXEL_ATTACH = 21;
	// > neoPixelSetAnimation/deviceId/animation/red/green/blue/b16 speed
	public final static int NEO_PIXEL_SET_ANIMATION = 22;
	// > neoPixelWriteMatrix/deviceId/[] buffer
	public final static int NEO_PIXEL_WRITE_MATRIX = 23;
	// > analogWrite/pin/value
	public final static int ANALOG_WRITE = 24;
	// > digitalWrite/pin/value
	public final static int DIGITAL_WRITE = 25;
	// > disablePin/pin
	public final static int DISABLE_PIN = 26;
	// > disablePins
	public final static int DISABLE_PINS = 27;
	// > pinMode/pin/mode
	public final static int PIN_MODE = 28;
	// < publishAttachedDevice/deviceId/str deviceName
	public final static int PUBLISH_ATTACHED_DEVICE = 29;
	// < publishBoardStatus/b16 microsPerLoop/b16 sram/[] deviceSummary
	public final static int PUBLISH_BOARD_STATUS = 30;
	// < publishDebug/str debugMsg
	public final static int PUBLISH_DEBUG = 31;
	// < publishMessageAck/function
	public final static int PUBLISH_MESSAGE_ACK = 32;
	// < publishSensorData/deviceId/[] data
	public final static int PUBLISH_SENSOR_DATA = 33;
	// < publishServoEvent/deviceId/eventType/currentPos/targetPos
	public final static int PUBLISH_SERVO_EVENT = 34;
	// > setTrigger/pin/triggerValue
	public final static int SET_TRIGGER = 35;
	// > setDebounce/pin/delay
	public final static int SET_DEBOUNCE = 36;
	// > serialRelay/deviceId/serialPort/[] relayData
	public final static int SERIAL_RELAY = 37;
	// > servoAttach/deviceId/pin/initPos/b16 initVelocity
	public final static int SERVO_ATTACH = 38;
	// > servoEnablePwm/deviceId/pin
	public final static int SERVO_ENABLE_PWM = 39;
	// > servoDisablePwm/deviceId
	public final static int SERVO_DISABLE_PWM = 40;
	// > servoSetMaxVelocity/deviceId/b16 maxVelocity
	public final static int SERVO_SET_MAX_VELOCITY = 41;
	// > servoSetVelocity/deviceId/b16 velocity
	public final static int SERVO_SET_VELOCITY = 42;
	// > servoSweepStart/deviceId/min/max/step
	public final static int SERVO_SWEEP_START = 43;
	// > servoSweepStop/deviceId
	public final static int SERVO_SWEEP_STOP = 44;
	// > servoWrite/deviceId/target
	public final static int SERVO_WRITE = 45;
	// > servoWriteMicroseconds/deviceId/b16 ms
	public final static int SERVO_WRITE_MICROSECONDS = 46;

	/**
	 * These methods will be invoked from the Msg class as callbacks from
	 * MrlComm.
	 */

	// public void publishMRLCommError(String errorMsg/*str*/){}
	// public void publishBoardInfo(Integer version/*byte*/, Integer
	// boardType/*byte*/){}
	// public void publishEcho(Long sInt/*bu32*/){}
	// public void publishCustomMsg(int[] msg/*[]*/){}
	// public void publishAttachedDevice(Integer deviceId/*byte*/, String
	// deviceName/*str*/){}
	// public void publishBoardStatus(Integer microsPerLoop/*b16*/, Integer
	// sram/*b16*/, int[] deviceSummary/*[]*/){}
	// public void publishDebug(String debugMsg/*str*/){}
	// public void publishMessageAck(Integer function/*byte*/){}
	// public void publishSensorData(Integer deviceId/*byte*/, int[]
	// data/*[]*/){}
	// public void publishServoEvent(Integer deviceId/*byte*/, Integer
	// eventType/*byte*/, Integer currentPos/*byte*/, Integer
	// targetPos/*byte*/){}

	public transient final static Logger log = LoggerFactory.getLogger(Msg.class);

	public Msg(Arduino arduino, SerialDevice serial) {
		this.arduino = arduino;
		this.serial = serial;
	}

	// transient private Msg instance;

	// ArduinoSerialCallBacks - TODO - extract interface
	transient private Arduino arduino;

	transient private SerialDevice serial;

	/**
	 * want to grab it when SerialDevice is created
	 *
	 * @param serial
	 * @return
	 */
	/*
	 * static public synchronized Msg getInstance(Arduino arduino, SerialDevice
	 * serial) { if (instance == null) { instance = new Msg(); }
	 * 
	 * instance.arduino = arduino; instance.serial = serial;
	 * 
	 * return instance; }
	 */

	public void processCommand(int[] ioCmd) {
		int startPos = 0;
		int method = ioCmd[startPos];
		switch (method) {
		case PUBLISH_MRLCOMM_ERROR: {
			String errorMsg = str(ioCmd, startPos + 2, ioCmd[startPos + 1]);
			startPos += 1 + ioCmd[startPos + 1];

			arduino.invoke("publishMRLCommError", errorMsg);

			// arduino.publishMRLCommError( errorMsg);

			break;
		}
		case PUBLISH_BOARD_INFO: {
			Integer version = ioCmd[startPos + 1]; // bu8
			startPos += 1;
			Integer boardType = ioCmd[startPos + 1]; // bu8
			startPos += 1;

			arduino.invoke("publishBoardInfo", version, boardType);

			// arduino.publishBoardInfo( version, boardType);

			break;
		}
		case PUBLISH_ECHO: {
			Long sInt = bu32(ioCmd, startPos + 1);
			startPos += 4; // bu32

			arduino.invoke("publishEcho", sInt);

			// arduino.publishEcho( sInt);

			break;
		}
		case PUBLISH_CUSTOM_MSG: {
			int[] msg = subArray(ioCmd, startPos + 2, ioCmd[startPos + 1]);
			startPos += 1 + ioCmd[startPos + 1];

			arduino.invoke("publishCustomMsg", msg);

			// arduino.publishCustomMsg( msg);

			break;
		}
		case PUBLISH_ATTACHED_DEVICE: {
			Integer deviceId = ioCmd[startPos + 1]; // bu8
			startPos += 1;
			String deviceName = str(ioCmd, startPos + 2, ioCmd[startPos + 1]);
			startPos += 1 + ioCmd[startPos + 1];

			arduino.invoke("publishAttachedDevice", deviceId, deviceName);

			// arduino.publishAttachedDevice( deviceId, deviceName);

			break;
		}
		case PUBLISH_BOARD_STATUS: {
			Integer microsPerLoop = b16(ioCmd, startPos + 1);
			startPos += 2; // b16
			Integer sram = b16(ioCmd, startPos + 1);
			startPos += 2; // b16
			int[] deviceSummary = subArray(ioCmd, startPos + 2, ioCmd[startPos + 1]);
			startPos += 1 + ioCmd[startPos + 1];

			arduino.invoke("publishBoardStatus", microsPerLoop, sram, deviceSummary);

			// arduino.publishBoardStatus( microsPerLoop, sram, deviceSummary);

			break;
		}
		case PUBLISH_DEBUG: {
			String debugMsg = str(ioCmd, startPos + 2, ioCmd[startPos + 1]);
			startPos += 1 + ioCmd[startPos + 1];

			arduino.invoke("publishDebug", debugMsg);

			// arduino.publishDebug( debugMsg);

			break;
		}
		case PUBLISH_MESSAGE_ACK: {
			Integer function = ioCmd[startPos + 1]; // bu8
			startPos += 1;

			arduino.invoke("publishMessageAck", function);

			// arduino.publishMessageAck( function);

			break;
		}
		case PUBLISH_SENSOR_DATA: {
			Integer deviceId = ioCmd[startPos + 1]; // bu8
			startPos += 1;
			int[] data = subArray(ioCmd, startPos + 2, ioCmd[startPos + 1]);
			startPos += 1 + ioCmd[startPos + 1];

			arduino.invoke("publishSensorData", deviceId, data);

			// arduino.publishSensorData( deviceId, data);

			break;
		}
		case PUBLISH_SERVO_EVENT: {
			Integer deviceId = ioCmd[startPos + 1]; // bu8
			startPos += 1;
			Integer eventType = ioCmd[startPos + 1]; // bu8
			startPos += 1;
			Integer currentPos = ioCmd[startPos + 1]; // bu8
			startPos += 1;
			Integer targetPos = ioCmd[startPos + 1]; // bu8
			startPos += 1;

			arduino.invoke("publishServoEvent", deviceId, eventType, currentPos, targetPos);

			// arduino.publishServoEvent( deviceId, eventType, currentPos,
			// targetPos);

			break;
		}

		}

	}

	// Java-land --to--> MrlComm

	public void getBoardInfo() {
		try {
			write(MAGIC_NUMBER);
			write(1); // size
			write(GET_BOARD_INFO); // msgType = 2

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void enableBoardStatus(Boolean enabled/* bool */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1); // size
			write(ENABLE_BOARD_STATUS); // msgType = 4
			writebool(enabled);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void enableHeartbeat(Boolean enabled/* bool */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1); // size
			write(ENABLE_HEARTBEAT); // msgType = 5
			writebool(enabled);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void enablePin(Integer address/* byte */, Integer type/* byte */, Integer rate/* b16 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + 2); // size
			write(ENABLE_PIN); // msgType = 6
			write(address);
			write(type);
			writeb16(rate);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void heartbeat() {
		try {
			write(MAGIC_NUMBER);
			write(1); // size
			write(HEARTBEAT); // msgType = 7

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void setDebug(Boolean enabled/* bool */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1); // size
			write(SET_DEBUG); // msgType = 8
			writebool(enabled);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void setSerialRate(Integer rate/* b32 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 4); // size
			write(SET_SERIAL_RATE); // msgType = 9
			writeb32(rate);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void softReset() {
		try {
			write(MAGIC_NUMBER);
			write(1); // size
			write(SOFT_RESET); // msgType = 10

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void echo(Long sInt/* bu32 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 4); // size
			write(ECHO); // msgType = 11
			writebu32(sInt);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void controllerAttach(Integer serialPort/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1); // size
			write(CONTROLLER_ATTACH); // msgType = 13
			write(serialPort);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void customMsg(int[] msg/* [] */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + (1 + msg.length)); // size
			write(CUSTOM_MSG); // msgType = 14
			write(msg);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void deviceDetach(Integer deviceId/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1); // size
			write(DEVICE_DETACH); // msgType = 16
			write(deviceId);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void i2cAttach(Integer deviceId/* byte */, Integer i2cBus/* byte */, Integer deviceType/* byte */, Integer deviceAddress/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + 1 + 1); // size
			write(I2C_ATTACH); // msgType = 17
			write(deviceId);
			write(i2cBus);
			write(deviceType);
			write(deviceAddress);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void i2cRead(Integer deviceId/* byte */, Integer deviceAddress/* byte */, Integer size/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + 1); // size
			write(I2C_READ); // msgType = 18
			write(deviceId);
			write(deviceAddress);
			write(size);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void i2cWrite(Integer deviceId/* byte */, Integer deviceAddress/* byte */, int[] data/* [] */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + (1 + data.length)); // size
			write(I2C_WRITE); // msgType = 19
			write(deviceId);
			write(deviceAddress);
			write(data);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void i2cWriteRead(Integer deviceId/* byte */, Integer deviceAddress/* byte */, Integer readSize/* byte */, Integer writeValue/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + 1 + 1); // size
			write(I2C_WRITE_READ); // msgType = 20
			write(deviceId);
			write(deviceAddress);
			write(readSize);
			write(writeValue);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void neoPixelAttach(Integer deviceId/* byte */, Integer pin/* byte */, Integer numPixels/* b32 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + 4); // size
			write(NEO_PIXEL_ATTACH); // msgType = 21
			write(deviceId);
			write(pin);
			writeb32(numPixels);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void neoPixelSetAnimation(Integer deviceId/* byte */, Integer animation/* byte */, Integer red/* byte */, Integer green/* byte */, Integer blue/* byte */,
			Integer speed/* b16 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + 1 + 1 + 1 + 2); // size
			write(NEO_PIXEL_SET_ANIMATION); // msgType = 22
			write(deviceId);
			write(animation);
			write(red);
			write(green);
			write(blue);
			writeb16(speed);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void neoPixelWriteMatrix(Integer deviceId/* byte */, int[] buffer/* [] */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + (1 + buffer.length)); // size
			write(NEO_PIXEL_WRITE_MATRIX); // msgType = 23
			write(deviceId);
			write(buffer);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void analogWrite(Integer pin/* byte */, Integer value/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1); // size
			write(ANALOG_WRITE); // msgType = 24
			write(pin);
			write(value);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void digitalWrite(Integer pin/* byte */, Integer value/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1); // size
			write(DIGITAL_WRITE); // msgType = 25
			write(pin);
			write(value);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void disablePin(Integer pin/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1); // size
			write(DISABLE_PIN); // msgType = 26
			write(pin);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void disablePins() {
		try {
			write(MAGIC_NUMBER);
			write(1); // size
			write(DISABLE_PINS); // msgType = 27

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void pinMode(Integer pin/* byte */, Integer mode/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1); // size
			write(PIN_MODE); // msgType = 28
			write(pin);
			write(mode);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void setTrigger(Integer pin/* byte */, Integer triggerValue/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1); // size
			write(SET_TRIGGER); // msgType = 35
			write(pin);
			write(triggerValue);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void setDebounce(Integer pin/* byte */, Integer delay/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1); // size
			write(SET_DEBOUNCE); // msgType = 36
			write(pin);
			write(delay);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void serialRelay(Integer deviceId/* byte */, Integer serialPort/* byte */, int[] relayData/* [] */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + (1 + relayData.length)); // size
			write(SERIAL_RELAY); // msgType = 37
			write(deviceId);
			write(serialPort);
			write(relayData);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoAttach(Integer deviceId/* byte */, Integer pin/* byte */, Integer initPos/* byte */, Integer initVelocity/* b16 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + 1 + 2); // size
			write(SERVO_ATTACH); // msgType = 38
			write(deviceId);
			write(pin);
			write(initPos);
			writeb16(initVelocity);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoEnablePwm(Integer deviceId/* byte */, Integer pin/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1); // size
			write(SERVO_ENABLE_PWM); // msgType = 39
			write(deviceId);
			write(pin);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoDisablePwm(Integer deviceId/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1); // size
			write(SERVO_DISABLE_PWM); // msgType = 40
			write(deviceId);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoSetMaxVelocity(Integer deviceId/* byte */, Integer maxVelocity/* b16 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 2); // size
			write(SERVO_SET_MAX_VELOCITY); // msgType = 41
			write(deviceId);
			writeb16(maxVelocity);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoSetVelocity(Integer deviceId/* byte */, Integer velocity/* b16 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 2); // size
			write(SERVO_SET_VELOCITY); // msgType = 42
			write(deviceId);
			writeb16(velocity);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoSweepStart(Integer deviceId/* byte */, Integer min/* byte */, Integer max/* byte */, Integer step/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1 + 1 + 1); // size
			write(SERVO_SWEEP_START); // msgType = 43
			write(deviceId);
			write(min);
			write(max);
			write(step);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoSweepStop(Integer deviceId/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1); // size
			write(SERVO_SWEEP_STOP); // msgType = 44
			write(deviceId);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoWrite(Integer deviceId/* byte */, Integer target/* byte */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 1); // size
			write(SERVO_WRITE); // msgType = 45
			write(deviceId);
			write(target);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public void servoWriteMicroseconds(Integer deviceId/* byte */, Integer ms/* b16 */) {
		try {
			write(MAGIC_NUMBER);
			write(1 + 1 + 2); // size
			write(SERVO_WRITE_MICROSECONDS); // msgType = 46
			write(deviceId);
			writeb16(ms);

		} catch (Exception e) {
			serial.error(e);
		}
	}

	public String str(int[] buffer, int start, int size) {
		byte[] b = new byte[size];
		for (int i = start; i < start + size; ++i) {
			b[i - start] = (byte) (buffer[i] & 0xFF);
		}
		return new String(b);
	}

	public int[] subArray(int[] buffer, int start, int size) {
		return Arrays.copyOfRange(buffer, start, start + size);
	}

	// signed 16 bit bucket
	public int b16(int[] buffer, int start/* =0 */) {
		return (short) (buffer[start] << 8) + buffer[start + 1];
	}

	// signed 32 bit bucket
	public int b32(int[] buffer, int start/* =0 */) {
		return ((buffer[start + 0] << 24) + (buffer[start + 1] << 16) + (buffer[start + 2] << 8) + buffer[start + 3]);
	}

	// unsigned 32 bit bucket
	public long bu32(int[] buffer, int start/* =0 */) {
		long ret = ((buffer[start + 0] << 24) + (buffer[start + 1] << 16) + (buffer[start + 2] << 8) + buffer[start + 3]);
		if (ret < 0) {
			return 4294967296L + ret;
		}

		return ret;
	}

	void write(int b8) throws Exception {

		if ((b8 < 0) || (b8 > 255)) {
			log.error("writeByte overrun - should be  0 <= value <= 255 - value = {}", b8);
		}

		serial.write(b8 & 0xFF);
	}

	void writebool(boolean b1) throws Exception {
		if (b1) {
			serial.write(1);
		} else {
			serial.write(0);
		}
	}

	void writeb16(int b16) throws Exception {
		if ((b16 < -32768) || (b16 > 32767)) {
			log.error("writeByte overrun - should be  -32,768 <= value <= 32,767 - value = {}", b16);
		}

		write(b16 >> 8 & 0xFF);
		write(b16 & 0xFF);
	}

	void writeb32(int b32) throws Exception {
		write(b32 >> 24 & 0xFF);
		write(b32 >> 16 & 0xFF);
		write(b32 >> 8 & 0xFF);
		write(b32 & 0xFF);
	}

	void writebu32(long b32) throws Exception {
		write((int) (b32 >> 24 & 0xFF));
		write((int) (b32 >> 16 & 0xFF));
		write((int) (b32 >> 8 & 0xFF));
		write((int) (b32 & 0xFF));
	}

	void write(String str) throws Exception {
		write(str.getBytes());
	}

	void write(int[] array) throws Exception {
		// write size
		write(array.length & 0xFF);

		// write data
		for (int i = 0; i < array.length; ++i) {
			write(array[i] & 0xFF);
		}
	}

	void write(byte[] array) throws Exception {
		// write size
		write(array.length);

		// write data
		for (int i = 0; i < array.length; ++i) {
			write(array[i]);
		}
	}

	public static void main(String[] args) {
		try {

			// FIXME - Test service started or reference retrieved
			// FIXME - subscribe to publishError
			// FIXME - check for any error
			// FIXME - basic design - expected state is connected and ready -
			// between classes it
			// should connect - also dumping serial comm at different levels so
			// virtual arduino in
			// Python can model "real" serial comm
			String port = "COM10";

			LoggingFactory.init(Level.INFO);

			/*
			 * Runtime.start("gui","GUIService"); VirtualArduino virtual =
			 * (VirtualArduino)Runtime.start("varduino","VirtualArduino");
			 * virtual.connectVirtualUart(port, port + "UART");
			 */

			Arduino arduino = (Arduino) Runtime.start("arduino", "Arduino");
			arduino.connect(port);
			arduino.enableBoardStatus(true);
			arduino.enableBoardStatus(false);
			// arduino.test();

			Servo servo01 = (Servo) Runtime.start("servo01", "Servo");
			servo01.attach(arduino, 8);

			servo01.moveTo(30);
			servo01.moveTo(130);

		} catch (Exception e) {
			log.error("main threw", e);
		}

	}

}
