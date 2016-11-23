package org.myrobotlab.service;

import static org.myrobotlab.arduino.Msg.DEVICE_TYPE_ARDUINO;
import static org.myrobotlab.arduino.Msg.DEVICE_TYPE_I2C;
import static org.myrobotlab.arduino.Msg.DEVICE_TYPE_MOTOR;
import static org.myrobotlab.arduino.Msg.DEVICE_TYPE_NEOPIXEL;
import static org.myrobotlab.arduino.Msg.DEVICE_TYPE_SERVO;
import static org.myrobotlab.arduino.Msg.DEVICE_TYPE_ULTRASONIC;
import static org.myrobotlab.arduino.Msg.MAGIC_NUMBER;
import static org.myrobotlab.arduino.Msg.MAX_MSG_SIZE;
import static org.myrobotlab.arduino.Msg.MRLCOMM_VERSION;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.myrobotlab.arduino.ArduinoUtils;
import org.myrobotlab.arduino.BoardInfo;
import org.myrobotlab.arduino.BoardStatus;
//import org.myrobotlab.arduino.MrlMsg;
import org.myrobotlab.arduino.Msg;
import org.myrobotlab.codec.serial.ArduinoMsgCodec;
import org.myrobotlab.framework.Service;
import org.myrobotlab.framework.ServiceType;
import org.myrobotlab.i2c.I2CBus;
import org.myrobotlab.io.FileIO;
import org.myrobotlab.logging.Level;
import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.logging.Logging;
import org.myrobotlab.logging.LoggingFactory;
import org.myrobotlab.motor.MotorConfig;
import org.myrobotlab.motor.MotorConfigDualPwm;
import org.myrobotlab.motor.MotorConfigPulse;
import org.myrobotlab.motor.MotorConfigSimpleH;
import org.myrobotlab.service.data.DeviceMapping;
import org.myrobotlab.service.data.Pin;
import org.myrobotlab.service.data.PinData;
import org.myrobotlab.service.data.SensorData;
import org.myrobotlab.service.interfaces.DeviceControl;
import org.myrobotlab.service.interfaces.DeviceController;
import org.myrobotlab.service.interfaces.I2CBusControl;
import org.myrobotlab.service.interfaces.I2CBusController;
import org.myrobotlab.service.interfaces.I2CControl;
import org.myrobotlab.service.interfaces.I2CController;
import org.myrobotlab.service.interfaces.Microcontroller;
import org.myrobotlab.service.interfaces.MotorControl;
import org.myrobotlab.service.interfaces.MotorController;
import org.myrobotlab.service.interfaces.NeoPixelControl;
import org.myrobotlab.service.interfaces.NeoPixelController;
import org.myrobotlab.service.interfaces.PinArrayControl;
import org.myrobotlab.service.interfaces.PinArrayListener;
import org.myrobotlab.service.interfaces.PinDefinition;
import org.myrobotlab.service.interfaces.PinListener;
import org.myrobotlab.service.interfaces.RecordControl;
import org.myrobotlab.service.interfaces.SensorController;
import org.myrobotlab.service.interfaces.SensorDataListener;
import org.myrobotlab.service.interfaces.SensorDataPublisher;
import org.myrobotlab.service.interfaces.SerialDataListener;
import org.myrobotlab.service.interfaces.ServoControl;
import org.myrobotlab.service.interfaces.ServoController;
import org.myrobotlab.service.interfaces.UltrasonicSensorControl;
import org.slf4j.Logger;

/**
 * Implementation of a Arduino Service connected to MRL through a serial port.
 * The protocol is basically a pass through of system calls to the Arduino
 * board. Data can be passed back from the digital or analog ports by request to
 * start polling. The serial port can be wireless (bluetooth), rf, or wired. The
 * communication protocol supported is in MRLComm.ino
 *
 * Should support nearly all Arduino board types
 *
 * digitalRead() works on all pins. It will just round the analog value received
 * and present it to you. If analogRead(A0) is greater than or equal to 512,
 * digitalRead(A0) will be 1, else 0. digitalWrite() works on all pins, with
 * allowed parameter 0 or 1. digitalWrite(A0,0) is the same as
 * analogWrite(A0,0), and digitalWrite(A0,1) is the same as analogWrite(A0,255)
 * analogRead() works only on analog pins. It can take any value between 0 and
 * 1023. analogWrite() works on all analog pins and all digital PWM pins. You
 * can supply it any value between 0 and 255
 *
 * TODO - make microcontroller interface - getPins digitalWrite analogWrite
 * writeMicroseconds pinMode etc.. TODO - remove all non-microcontroller methods
 * TODO - call-back parseData() from serial service --> to MicroController - so
 * microcontoller can parse format messages to universal REST format
 *
 * TODO - set trigger in combination of polling should be a universal
 * microcontroller function
 *
 *
 * // need a method to identify type of board //
 * http://forum.arduino.cc/index.php?topic=100557.0
 *
 * public static final int STEPPER_EVENT_STOP = 1; public static final int
 * STEPPER_TYPE_POLOLU = 1; public static final int CUSTOM_MSG = 50;
 *
 * FUTURE UPLOADS
 * https://pragprog.com/magazines/2011-04/advanced-arduino-hacking
 *
 */

/**
 *
 * Interface Design Mantra
 *
 * MRL runs on a computer. An Arduino is a Mircro controller. For all the things
 * in MRL which need an Arduino - there is a physical connection. E.g Servo
 * --plugs into--> Arduino --plugs into--> Computer running MRL or Motor --plugs
 * into--> Arduino --plugs into--> Computer running MRL
 *
 * so in short - the communication between these services Motor & Arduino or
 * Servo & Arduino can be optimized, because the services will never be remote
 * from one another.
 *
 * The whole publish, invoke subscribe messaging system works great, is fairly
 * efficient, and can work remotely. But an optimization here might be a good
 * thing if we have to route data from Serial -> Arduino -> Motor -> WebGui ->
 * Angular UI !
 *
 * We will use standard Java callback Listener patterns. It should enforce the
 * methods needed from appropriate interfaces.
 *
 * Arduino will have maps of other services it currently needs to callback to.
 * It possibly will have wrapper classes around those services in order to
 * prevent serialization issues (with the actual service marked as transient)
 *
 * If the "controller" is marked as transient in object which is attached - this
 * possibly will fix cyclical serialization issues
 *
 */

public class Arduino extends Service implements Microcontroller, PinArrayControl, I2CBusController, I2CController, SerialDataListener, ServoController, MotorController,
		NeoPixelController, SensorDataPublisher, DeviceController, SensorController, SensorDataListener, RecordControl {

	public static class AckLock {
		volatile boolean acknowledged = false;
	}

	public static class I2CDeviceMap {
		public int busAddress;
		public transient I2CControl control;
		public int deviceAddress;
	}

	public static class Sketch implements Serializable {
		private static final long serialVersionUID = 1L;
		public String data;
		public String name;

		public Sketch(String name, String data) {
			this.name = name;
			this.data = data;
		}
	}
	public transient static final int BOARD_TYPE_ID_ADK_MEGA = 3;
	public transient static final int BOARD_TYPE_ID_MEGA = 1;
	// MrlComm definition
	public transient static final int BOARD_TYPE_ID_UNKNOWN = 0;

	public transient static final int BOARD_TYPE_ID_UNO = 2;
	public transient static final String BOARD_TYPE_MEGA = "mega";
	public transient static final String BOARD_TYPE_MEGA_ADK = "megaADK";

	public transient static final String BOARD_TYPE_UNO = "uno";

	/**
	 * FIXME ! - these processor types ! - something we are not interested in
	 * and do not have to deal with - we are far more interested in
	 * NUM_DIGITAL_PINS and "board pin layouts" -
	 *
	 * As far as I can tell board types are in variants 1.0.5 Arduino IDE
	 * includes
	 *
	 * This is the best reference I have found regarding actual pin capabilities
	 * https://learn.sparkfun.com/tutorials/arduino-comparison-guide#totally-
	 * tabular Uno & Duemilanove have 14 digital pins (6 PWM) & 6 analog - total
	 * 20 Mini & Pro have 14 digital pins (8 PWM) & 6 analog - total 20
	 *
	 * ATmega328 Boards 32kB Program Space // 1 UART // 6 PWM // 4-8 Analog
	 * Inputs // 9-14 Digital I/O ATmega2560 Arduino Mega's 256kB Program Space
	 * // 4 UARTs // 14 PWM // 16 Analog Inputs // 54 Digital I/O -
	 *
	 * So at the moment .. there is only Uno & Mega !!!
	 * 
	 * With the new upload method, Gael need to have support for ADK Mega
	 *
	 */

	// Java-land defintion

	public static final int FALSE = 0;
	public static final int HIGH = 0x1;
	public static final int INPUT = 0x0;

	public transient final static Logger log = LoggerFactory.getLogger(Arduino.class);
	public static final int LOW = 0x0;
	public static final int MOTOR_BACKWARD = 0;
	public static final int MOTOR_FORWARD = 1;

	public static final int MRL_IO_NOT_DEFINED = 0;
	public static final int MRL_IO_SERIAL_0 = 1;
	public static final int MRL_IO_SERIAL_1 = 2;
	public static final int MRL_IO_SERIAL_2 = 3;

	public static final int MRL_IO_SERIAL_3 = 4;

	public static final int OUTPUT = 0x1;

	private static final long serialVersionUID = 1L;

	public static final int TRUE = 1;

	/**
	 * This static method returns all the details of the class without it having
	 * to be constructed. It has description, categories, dependencies, and peer
	 * definitions.
	 *
	 * @return ServiceType - returns all the data
	 *
	 */
	static public ServiceType getMetaData() {

		ServiceType meta = new ServiceType(Arduino.class.getCanonicalName());
		meta.addDescription("This service interfaces with an Arduino micro-controller");
		meta.addCategory("microcontroller");
		meta.addPeer("serial", "Serial", "serial device for this Arduino");
		return meta;
	}

	static String intsToString(int[] ints) {
		return intsToString(ints, 0, ints.length);
	}

	static String intsToString(int[] ints, int begin) {
		return intsToString(ints, begin, ints.length - begin);
	}

	static String intsToString(int[] ints, int begin, int length) {
		byte[] b = new byte[length];
		for (int i = 0; i < length; ++i) {
			b[i] = (byte) ints[begin + i];
		}
		return new String(b);
	}

	public static void main(String[] args) {
		try {

			LoggingFactory.init(Level.INFO);

			/*
			 * InMoov i01 = (InMoov)Runtime.start("i01", "InMoov");
			 * VirtualDevice virtual = (VirtualDevice)Runtime.start("virtual",
			 * "VirtualDevice"); virtual.createVirtualSerial("COM7");
			 * 
			 * String leftPort = "COM5"; String rightPort = "COM7";
			 * i01.startAll(leftPort, rightPort);
			 * 
			 * InMoovTorso torso = i01.startTorso(leftPort);
			 * i01.torso.topStom.detach(); i01.torso.topStom.attach("i01.left",
			 * 49);
			 */

			Arduino arduino = (Arduino) Runtime.start("arduino", "Arduino");
			Serial serial = arduino.getSerial();
			// Runtime.start("gui", "GUIService");
			List<String> ports = serial.getPortNames();
			log.info(Arrays.toString(ports.toArray()));
			arduino.setBoardMega();
			// log.info(arduino.getBoardType());
			// if connect - possibly you can set the board type correctly
			arduino.getBoardInfo();
			arduino.connect("COM4");

			arduino.uploadSketch("C:\\tools\\arduino-1.6.9");

			Servo servo = (Servo) Runtime.start("servo", "Servo");
			Runtime.start("gui", "GUIService");
			servo.attach(arduino, 7);
			// servo.detach(arduino);
			servo.attach(9);

			// servo.detach(arduino);
			// arduino.servoDetach(servo); Arduino power save - "detach()"

			servo.moveTo(0);
			servo.moveTo(180);
			servo.setInverted(true);
			servo.moveTo(0);
			servo.moveTo(180);
			servo.setInverted(true);
			servo.moveTo(0);
			servo.moveTo(180);
			// arduino.attachDevice(servo, null);
			// servo.attach();
			int angle = 0;
			int max = 5000;
			while (true) {
				// System.out.println(angle);
				angle++;
				servo.moveTo(angle % 180);
				if (angle > max) {
					break;
				}
			}
			System.out.println("done with loop..");
			log.info("here");

		} catch (Exception e) {
			Logging.logError(e);
		}
	}

	boolean ackEnabled = false;

	// make sure this is sync'd across threads,
	transient final private AckLock ackLock = new AckLock();

	/**
	 * path of the Arduino IDE must be set by user
	 */
	public String arduinoPath;

	transient HashMap<Integer, Arduino> attachedController = new HashMap<Integer, Arduino>();

	/**
	 * board info "from" MrlComm - which can be different from what the user
	 * say's it is - if there is a difference the "user" should be notified -
	 * but not forced to use the mrlBoardInfo.
	 */
	final BoardInfo boardInfo = new BoardInfo();

	/**
	 * board type - UNO Mega etc..
	 * 
	 * if the user 'connects' first then the info could come from the board ..
	 * but if the user wants to upload first a npe will be thrown so we default
	 * it here to Uno
	 */
	public String boardType = null;

	int byteCount;
	transient ArduinoMsgCodec codec = new ArduinoMsgCodec();

	public transient int controllerAttachAs = MRL_IO_NOT_DEFINED;

	/**
	 * number of ms to pause after sending a message to the Arduino
	 */
	public int delay = 0;

	/**
	 * id reference of sensor, key is the MRLComm device id
	 */
	transient HashMap<Integer, DeviceMapping> deviceIndex = new HashMap<Integer, DeviceMapping>();

	/**
	 * Devices - string name index of device we need 2 indexes for sensors
	 * because they will be referenced by name OR by index
	 */
	transient HashMap<String, DeviceMapping> deviceList = new HashMap<String, DeviceMapping>();

	int error_arduino_to_mrl_rx_cnt = 0;

	int error_mrl_to_arduino_rx_cnt = 0;

	boolean heartbeat = false;

	/**
	 * <pre>
	 * Hi Mats : I would recommend using the deviceList instead of creating another
	 * map here .. the deviceList contains DeviceMapping & DeviceMappings have Object[] config
	 * you can add your I2CDeviceMap to config - because it would be "config" for your
	 * DeviceControl.DEVICE_TYPE_I2C
	 * 
	 * I think it might be could to pull as much I2C definitions as possible - what I mean to say
	 * is I2C is its "own" thing - for example I2CDeviceMap exists currently in both Arduino & Raspi and
	 * its the same definition - it deserves to exist on its own (perhaps in its own package name?) and
	 * Arduino & RasPi services should share that definition..
	 * 
	 * A I2CDeviceController interface should be defined too ..
	 * the signatures would follow the same pattern as all DeviceController & DeviceControl interfaces..
	 * 
	 * You already have a I2CControl which should be derived from DeviceControl..
	 * 
	 * You needs an I2CController (derived from DeviceController) -
	 * 		the controllers purpose is to implement the low level details so I2Control methods can be done
	 * 
	 * I2Control methods are :
	 *  	void createI2cDevice(int busAddress, int deviceAddress); // pure i2c no serviceName
	 *  	void releaseI2cDevice(int busAddress, int deviceAddress);
	 *  	void i2cWrite(int busAddress, int deviceAddress, byte[] buffer, int size);
	 *  	int i2cRead(int busAddress, int deviceAddress, byte[] buffer, int size);
	 * 
	 * I2CController methods could be
	 *  	void createI2cDevice(I2Control control, int busAddress, int deviceAddress); // don't need serviceName - you have the whole service in "control" parameter
	 *  	void releaseI2cDevice(I2Control control, int busAddress, int deviceAddress);
	 *  	void i2cWrite(I2Control control, int busAddress, int deviceAddress, byte[] buffer, int size);
	 *  	int i2cRead(I2Control control, int busAddress, int deviceAddress, byte[] buffer, int size);
	 * 
	 * The I2CControl method implementation would be potentially very simple - they would just call the controller's method with
	 * "this" as a first parameter... e.g.
	 * 
	 * 		void createI2cDevice(int busAddress, int deviceAddress, String serviceName){
	 * 					controller.createI2cDevice(this, busAddress, deviceAddress)
	 * 		}
	 * 
	 * This pattern follows MotorController / MotorControl & ServoController / ServoControl .. & Soon to be
	 * PinArrayController / PinArrayControl, SerialController / SerialControl
	 * 
	 * This abstraction is structured enough to follow, yet necessary to provide the implementation differences between say
	 * RasPi & Arduino for I2C ... or Serial control .. e.g. using a Serial service to read & write over USB vs Using Arduino
	 * to create and relay serial reads & writes over a different set of pins (Mega pins 14-19)
	 * 
	 * The plans I have for SerialControl & SerialController (not yet defined - but similar to I2cControll & I2cController) is
	 * SerialControl provides basic reads & writes - the SerialController provides the same methods, but with the SerialControl
	 * as the first parameter - Arduino would be a service which implemented both SerialControl & SerialController
	 * 
	 * A service which "needed" Serial control .. say a NeoPixel Ring would allow attachment to a Serial service directly through
	 * FTDI (https://github.com/tdicola/Adafruit_NeoPixel_FTDI)  or attachment to an Arduino Mega on pin 14 & 15.
	 * The business logic of driving the NeoPixel with serial commands would be in the NeoPixel service.
	 * 
	 *  GroG
	 * 
	 *  Comments on the above from Mats
	 *  Hi GroG.
	 * 
	 *  Thanks for writing and explaining about the responsibilities for the different interfaces.
	 *  I understand now that I have made a mistake when implementing I2CControl in RasPi, Ardino and I2cMux.
	 *  They all should implement I2CController
	 *  The services for i2c devices like Adafruit16CServoDriver, AdafruitIna219, I2cMux and Mpu6050 should implement I2CControl.
	 *  I will rework that so that everything follows the same pattern.
	 * 
	 *  About using the deviceList.
	 *  My understanding was/is that adding a device to the devicelist also would create a corresponding device in MRLComm.
	 *  Since a single i2c bus may contain as many as 127 addressable devices that would potentially use a lot of memory in MRLComm.
	 *  So I want to create a new device that represents a single I2CBus and that device should be added to the devicelist and also be created in
	 *  MRLComm.
	 *  That is the reason that I also created a new i2cDevices list to keep track of the different i2c devices.
	 * 
	 *  If we can add the i2c devices to the devicelist without creating a device in MRLComm, then that's perhaps a better way.
	 *  In that case we still need a i2cbus device that will be a MrlI2CDevice object.
	 *  To make it clear what it actually represents, I would like to rename it to MrlI2cBus.
	 * 
	 *  About I2CDeviceMap.
	 *  If we can add the i2c devices to the devicelist without creating a device in MRLComm then I2CDeviceMap isn't needed at all
	 *  in this Arduino service. So that's one option that needs to be explored / discussed.
	 * 
	 *  If that's not possible, then an other option is to keep the i2cDevices list using I2CDeviceMap.
	 *  I tried to use the same definition of it in both RasPi and Arduino.
	 *  However the I2CDeviceMap in RasPi is based on definitions in pi4j.
	 *  In the RasPi service both I2CBus and I2CDevice are objects defined in pi4j.
	 *  So I redefined I2CDeviceMap in this Arduino service to only use Strings, not objects.
	 *  They are not the same in this service and RasPi even if they share the same names.
	 * 
	 *  The third option is to keep the i2cDevices list but use DeviceMapping.
	 * 
	 *  I hope that I have been able to explain what I have done and the reasons for it.
	 *  So what way do you think is the best? Other people are also welcome to express their opinions.
	 * 
	 *  About I2CControl.
	 *  Since I2CControl should be the methods on the high level, I don't think that
	 *  the low level I2CController methods should be replicated.
	 *  It's one thing to use the I2CController method calls, something completley
	 *  different to implement them.
	 *  For example Arduino and RasPi implements the methods defined in the I2CController interface.
	 *  Adafruit16CServoController and Mpu6050 use the methods in the I2CController interface.
	 *  So I agree on the methods in I2CController, but not on the I2CControl methods that you suggested.
	 *  For example, I don't want a Python script create an Adafruit16CServoDriver and then
	 *  use Adafruit16CServoDriver.i2cWrite(...) to directly make i2c reads and writes.
	 *  That would be really strange.
	 *  I'm fine with a Python script using Arduino or RasPi to do i2c reads and writes since it
	 *  is defined in the I2CController interface.
	 *
	 * </pre>
	 */

	I2CBus i2cBus;

	volatile byte[] i2cData = new byte[64];
	// i2c This needs to be volatile because it will be updated in a different
	// threads
	volatile boolean i2cDataReturned = false;

	volatile int i2cDataSize;

	HashMap<String, I2CDeviceMap> i2cDevices = new HashMap<String, I2CDeviceMap>();

	transient int[] ioCmd = new int[MAX_MSG_SIZE];

	transient Msg msg;

	public int msgSize;

	Integer nextDeviceId = 0;

	private int numAck = 0;
	transient Map<String, PinArrayListener> pinArrayListeners = new HashMap<String, PinArrayListener>();

	int pinEventsDefaultRate = 8000;

	/**
	 * the definitive sequence of pins - "true address"
	 */
	Map<Integer, PinDefinition> pinIndex = null;

	/**
	 * map of pin listeners
	 */
	transient Map<Integer, List<PinListener>> pinListeners = new HashMap<Integer, List<PinListener>>();

	/**
	 * pin named map of all the pins on the board
	 */
	Map<String, PinDefinition> pinMap = null;
	/**
	 * the map of pins which the pin listeners are listening too - if the set is
	 * null they are listening to "any" published pin
	 */
	Map<String, Set<Integer>> pinSets = new HashMap<String, Set<Integer>>();
	transient FileOutputStream record = null;
	// for debuging & developing - need synchronized - both send & recv threads
	transient StringBuffer recordRxBuffer = new StringBuffer();
	transient StringBuffer recordTxBuffer = new StringBuffer();
	public int retryConnectDelay = 1500;

	// parameters for testing the getVersion retry stuff.
	// TODO: some way to do this more synchronously
	// perhaps when we connect to the serial port, MRLComm can just have the
	// version waiting?
	public int retryMax = 3;
	transient Arduino rootController = null;

	/**
	 * Serial service - the Arduino's serial connection
	 */
	Serial serial;
	public Sketch sketch;

	public String uploadSketchResult;

	public Arduino(String n) {
		super(n);
		serial = (Serial) createPeer("serial");
		createPinList();
		String mrlcomm = FileIO.resourceToString("Arduino/MrlComm/MRLComm.ino");
		setSketch(new Sketch("MRLComm", mrlcomm));

		// add self as an attached device
		// to handle pin events and other base
		// Arduino methods
		attachDevice(this, (Object[]) null);
	}

	public void analogWrite(int pin, int value) {
		log.info(String.format("analogWrite(%d,%d)", pin, value));
		msg.analogWrite(pin, value);
	}

	/**
	 * attach a pin listener which listens for an array of all active pins
	 */
	@Override
	public void attach(PinArrayListener listener) {
		pinArrayListeners.put(listener.getName(), listener);
	}

	/**
	 * attach a pin listener who listens to a specific pin FIXME - implement the
	 * 'specific' pin
	 */
	@Override
	public void attach(PinListener listener, int address) {
		String name = listener.getName();

		if (listener.isLocal()) {
			List<PinListener> list = null;
			if (pinListeners.containsKey(address)) {
				list = pinListeners.get(address);
			} else {
				list = new ArrayList<PinListener>();
			}
			list.add(listener);
			pinListeners.put(address, list);

		} else {
			// setup for pub sub
			// FIXME - there is an architectual problem here
			// locally it works - but remotely - outbox would need to know
			// specifics of
			// the data its sending
			addListener("publishPin", name, "onPin");
		}
	}

	/**
	 * String interface - this allows you to easily use url api requests like
	 * /attach/nameOfListener/3
	 */
	@Override
	public void attach(String listener, int address) {
		attach((PinListener) Runtime.getService(listener), address);
	}

	synchronized private Integer attachDevice(DeviceControl device, Object[] attachConfig) {
		DeviceMapping map = new DeviceMapping(this, attachConfig);
		map.setId(nextDeviceId);
		deviceList.put(device.getName(), map);
		deviceIndex.put(0, map);
		device.setController(this);
		++nextDeviceId;
		return map.getId();
	}

	// this allow to connect a controller to another controller with Serial1,
	// Serial2, Serial3 on a mega board
	public void connect(Arduino controller, String serialPort) throws IOException {
		if (controller == null) {
			error("setting null as controller");
			return;
		}
		if (controller == this) {
			error("controller can't attach to itself");
			return;
		}
		if (!controller.boardType.toLowerCase().contains("mega")) {
			error("You must connect to a Mega controller");
			return;
		}
		rootController = controller;
		// connect("COM15");
		serial = rootController.serial;
		switch (serialPort) {
		case "Serial1":
			controllerAttachAs = MRL_IO_SERIAL_1;
			break;
		case "Serial2":
			controllerAttachAs = MRL_IO_SERIAL_2;
			break;
		case "Serial3":
			controllerAttachAs = MRL_IO_SERIAL_3;
			break;
		default:
			error("Unknow serial port");
			return;
		}
		controller.controllerAttach(this, controllerAttachAs);
		// softReset();
		Integer version = getVersion();
		if (version == null || version != MRLCOMM_VERSION) {
			error("MRLComm expected version %d actual is %d", MRLCOMM_VERSION, version);
		}
		broadcastState();
	}

	public void connect(String port) {
		connect(port, Serial.BAUD_115200, 8, 1, 0);
	}

	/**
	 * default params to connect to Arduino & MRLComm.ino
	 *
	 * @param port
	 * @return
	 * @throws IOException
	 */
	@Override
	public void connect(String port, int rate, int databits, int stopbits, int parity) {

		try {

			serial.connect(port, rate, databits, stopbits, parity);

			// most likely on a real board this send will never get to
			// mrlcomm - because the board is not ready - but it doesnt hurt
			// and in fact it helps VirtualArduino - since we currently do not
			// have a DTR CDR line in the virtual port
			msg.getBoardInfo();

			log.info("waiting for boardInfo lock..........");
			synchronized (boardInfo) {
				try {
					boardInfo.wait(4500); // max wait 4.5 seconds - for port to
											// open
				} catch (InterruptedException e) {
				}
			}

			// we might be connected now
			// see what our version is like...
			Integer version = boardInfo.getVersion();

			if (version == null) {
				error("%s did not get response from arduino....", serial.getPortName());
			} else if (!version.equals(MRLCOMM_VERSION)) {
				error("MRLComm.ino responded with version %s expected version is %s", version, MRLCOMM_VERSION);
			} else {
				info("%s connected on %s responded version %s ... goodtimes...", serial.getName(), serial.getPortName(), version);
			}

		} catch (Exception e) {
			log.error("serial open threw", e);
			error(e.getMessage());
		}

		broadcastState();
	}

	// GroG asks DEPRECATE ???
	public void controllerAttach(Arduino controller, int serialPort) {
		attachedController.put(serialPort, controller);
		msg.controllerAttach(serialPort);
	}

	public Map<String, PinDefinition> createPinList() {
		pinMap = new HashMap<String, PinDefinition>();
		pinIndex = new HashMap<Integer, PinDefinition>();

		if (boardType != null && boardType.toLowerCase().contains("mega")) {
			for (int i = 0; i < 70; ++i) {
				PinDefinition pindef = new PinDefinition();
				String name = null;
				if (i == 0) {
					pindef.setRx(true);
				}
				if (i == 1) {
					pindef.setTx(true);
				}
				if (i < 1 || (i > 13 && i < 54)) {
					name = String.format("D%d", i);
					pindef.setDigital(true);
				} else if (i > 53) {
					name = String.format("A%d", i - 54);
					pindef.setAnalog(true);
				} else {
					name = String.format("D%d", i);
					pindef.setPwm(true);
				}
				pindef.setName(name);
				pindef.setAddress(i);
				pinMap.put(name, pindef);
				pinIndex.put(i, pindef);
			}
		} else {
			for (int i = 0; i < 20; ++i) {
				PinDefinition pindef = new PinDefinition();
				String name = null;
				if (i == 0) {
					pindef.setRx(true);
				}
				if (i == 1) {
					pindef.setTx(true);
				}
				if (i < 14) {
					name = String.format("D%d", i);
					pindef.setDigital(true);
				} else {
					pindef.setAnalog(true);
					name = String.format("A%d", i - 14);
				}
				if (i == 3 || i == 5 || i == 6 || i == 9 || i == 10 || i == 11) {
					pindef.setPwm(true);
					name = String.format("D%d", i);
				}
				pindef.setName(name);
				pindef.setAddress(i);
				pinMap.put(String.format("A%d", i), pindef);
				pinIndex.put(i, pindef);
			}
		}
		return pinMap;
	}

	public void customMsg(int... params) {
		msg.customMsg(params);
	}

	public synchronized void deviceControlAttach(DeviceControl device) {

		String name = device.getName();

		// check to see if we are already attached as the device controller
		// btw - this potentially will be a problem if its operating in a
		// different
		// process - the controller will probably be transient
		DeviceController dc = device.getController();
		if (dc == this) {
			log.info(String.format("%s already attached at device level - nothing to do", device.getName()));
			return;
		}

		// check to see if this device is already attached
		if (this != device.getController()) {
			device.setController(this);
		}

		// ??
		if (deviceList.containsKey(name)) {
			DeviceMapping map = deviceList.get(name);
			if (map.getId() == null) {
				log.error("oh no !  the device %s record is in place, but we do not have a device id - something wrong in mrlcomm?", name);
				log.error("mrlcomm is in an unknown state");
				throw new IllegalArgumentException("half attached device - name already defined, no id");
			}
		}

	}

	/*
	 * public void disableBoardStatus() { msg.disableBoardStatus(); }
	 */

	@Override
	public void deviceDetach(DeviceControl device) {
		log.info("detaching device {}", device.getName());
		msg.deviceDetach(getDeviceId(device));
	}

	/**
	 * silly Arduino implementation - but keeping it since its familiar
	 * 
	 * @param address
	 * @param value
	 */
	public void digitalWrite(int pin, int value) {
		log.info("digitalWrite {} {}", pin, value);
		msg.digitalWrite(pin, value);
		PinDefinition pinDef = pinIndex.get(pin); // why ?
		invoke("publishPinDefinition", pinDef);
	}

	public void disablePin(int address) {
		if (!isConnected()) {
			error("must be connected to disable pins");
			return;
		}
		msg.disablePin(address);
		PinDefinition pinDef = pinIndex.get(address);
		invoke("publishPinDefinition", pinDef);
	}

	public void disablePins() {
		msg.disablePins();
	}

	public void disconnect() {
		// boardInfo is not valid after disconnect
		// because we might be connecting to a different Arduino
		boardInfo.reset();
		for (Arduino controller : attachedController.values()) {
			controller.disconnect();
		}
		attachedController.clear();
		if (controllerAttachAs != MRL_IO_NOT_DEFINED) {
			controllerAttachAs = MRL_IO_NOT_DEFINED;
			serial = (Serial) createPeer("serial");
		} else {
			serial.disconnect();
		}
		broadcastState();
	}

	public void enableBoardStatus(Boolean enabled) {
		msg.enableBoardStatus(enabled);
	}

	public void enabledHeartbeat() {
		heartbeat = true;
		addTask("heartbeat", 1000, "heartbeat");
	}

	public void enablePin(int address) {
		enablePin(address, 0);
	}

	/**
	 * start polling reads of selected pin
	 *
	 * @param pin
	 * @throws Exception
	 */
	public void enablePin(int address, int rate) {
		if (!isConnected()) {
			error("must be connected to enable pins");
			return;
		}
		/*
		 * MrlMsg msg = new MrlMsg(ENABLE_PIN); msg.append(address); // ANALOG 1
		 * DIGITAL 0
		 * 
		 * 
		 * msg.append(getMrlPinType(pin)); // pinType // TODO - make this Hz so
		 * everyone is happy :) msg.append16(rate); // sendMsg(msg);
		 * 
		 * FIXME - why pinType ???
		 */

		PinDefinition pin = pinIndex.get(address);

		msg.enablePin(address, getMrlPinType(pin), rate);

		pin.setEnabled(true);
		invoke("publishPinDefinition", pin);
	}

	public BoardInfo getBoardInfo() {
		return boardInfo;
	}

	@Override
	public String getBoardType() {
		return boardType;
	}

	@Override
	public DeviceController getController() {
		return this;
	}

	Integer getDeviceId(DeviceControl device) {
		return getDeviceId(device.getName());
	}

	Integer getDeviceId(String name) {
		if (deviceList.containsKey(name)) {
			Integer id = deviceList.get(name).getId();
			if (id == null) {
				error("cannot get device id for %s - device attempetd to attach - but I suspect something went wrong", name);
			}
			return id;
		}
		log.error("getDeviceId could not find device {}", name);
		return null;
	}

	/**
	 * int identifier for MrlTypeDevice - this has to be in sync with MRLComm's
	 * type ids
	 *
	 * @param device
	 * @return
	 */
	// REMOVE IF NOT NEEDED !!!
	private Integer getMrlDeviceType(DeviceControl device) {

		// FIXME - this will be need to be more type specific
		if (device instanceof MotorControl) {
			return DEVICE_TYPE_MOTOR;
		}

		if (device instanceof Arduino) {
			return DEVICE_TYPE_ARDUINO;
		}

		// FixMe this does not follow spec..
		// of Control Controller
		if (device instanceof UltrasonicSensor) {
			return DEVICE_TYPE_ULTRASONIC;
		}

		if (device instanceof Servo) {
			return DEVICE_TYPE_SERVO;
		}

		if (device instanceof I2CBusControl) {
			return DEVICE_TYPE_I2C;
		}

		if (device instanceof NeoPixelControl) {
			return DEVICE_TYPE_NEOPIXEL;
		}

		throw new IllegalArgumentException(String.format("a mrl device type for %s of type %s could not be found ", device.getName(), device.getClass().getCanonicalName()));
	}

	/**
	 * int type to describe the pin defintion to Pin.h 0 digital 1 analog
	 * 
	 * @param pin
	 * @return
	 */
	public Integer getMrlPinType(PinDefinition pin) {
		if (boardType == null) {
			error("must have pin board type to determin pin definition");
			return null;
		}

		if (pin.isAnalog()) {
			return 1;
		}

		return 0;
	}

	@Override
	public List<PinDefinition> getPinList() {
		List<PinDefinition> list = new ArrayList<PinDefinition>(pinIndex.values());
		return list;
	}

	public String getPortName() {
		return serial.getPortName();
	}

	/**
	 * Use the serial service for serial activities ! No reason to replicate
	 * methods
	 *
	 * @return
	 */
	public Serial getSerial() {
		return serial;
	}

	public Sketch getSketch() {
		return sketch;
	}

	/**
	 * improved design for blocking async calls is in getBoardInfo()
	 * 
	 * @return
	 */
	@Deprecated // use getBoardInfo
	public Integer getVersion() {

		// version is dependent on board info
		if (boardInfo.isValid()) {
			return boardInfo.getVersion();
		}

		getBoardInfo();

		// broadcast state ??
		return boardInfo.getVersion();
	}

	public void heartbeat() {
		if (!heartbeat) {
			log.info("No answer from controller:{}. Disconnecting...", this.getName());
			purgeTask("heartbeat");
			if (isConnected()) {
				disconnect();
			}
		}
		heartbeat = false;
		msg.heartbeat();
	}

	@Override
	public void i2cAttach(I2CControl control, int busAddress, int deviceAddress) {
		// TODO Auto-generated method stub - I2C
		// Create the i2c bus device in MRLComm the first time this method is
		// invoked.
		// Add the i2c device to the list of i2cDevices
		// Pattern: deviceAttach(device, Object... config)
		// To add the i2c bus to the deviceList I need an device that represents
		// the i2c bus here and in MRLComm
		// This will only handle the creation of i2cBus.
		if (i2cBus == null) {
			i2cBus = new I2CBus(String.format("I2CBus%s", busAddress));
		}

		// deviceAttach(i2cBus, getMrlDeviceType(i2cBus), busAddress);
		// msg.i2cAttach(deviceId, getMrlDeviceType(i2cBus), deviceAddress);
		Integer deviceId = attachDevice(control, new Object[] { busAddress, deviceAddress });
		msg.i2cAttach(deviceId, busAddress, getMrlDeviceType(i2cBus), deviceAddress);

		// This part adds the service to the mapping between
		// busAddress||DeviceAddress
		// and the service name to be able to send data back to the invoker
		String key = String.format("%d.%d", busAddress, deviceAddress);
		I2CDeviceMap devicedata = new I2CDeviceMap();
		if (i2cDevices.containsKey(key)) {
			log.error(String.format("Device %s %s %s already exists.", busAddress, deviceAddress, control.getName()));
		} else {
			devicedata.busAddress = busAddress;
			devicedata.deviceAddress = deviceAddress;
			devicedata.control = control;
			i2cDevices.put(key, devicedata);
		}
	}

	@Override
	public int i2cRead(I2CControl control, int busAddress, int deviceAddress, byte[] buffer, int size) {
		i2cDataReturned = false;
		// Get the device index to the MRL i2c bus
		String i2cBus = String.format("I2CBus%s", busAddress);
		DeviceMapping map;
		map = deviceList.get(i2cBus);
		int id = map.getId(); // Device index to the I2CBus

		int retry = 0;
		int retryMax = 1000; // ( About 1000ms = s)
		try {
			/**
			 * We will wait up to retryMax times to get the i2c data back from
			 * MRLComm.c and wait 1 ms between each try. A blocking queue is not
			 * needed, as this is only a single data element - and blocking is
			 * not necessary.
			 */
			while ((retry < retryMax) && (!i2cDataReturned)) {
				sleep(1);
				++retry;
			}
		} catch (Exception e) {
			Logging.logError(e);
		}
		if (i2cDataReturned) {
			log.debug(String.format("i2cReturnData returned %s bytes to caller %s.", i2cDataSize, control.getName()));
			for (int i = 0; i < i2cDataSize; i++) {
				buffer[i] = i2cData[i];
				log.debug(String.format("i2cReturnData returned ix %s value %s", i, buffer[i]));
			}
			return i2cDataSize;
		}
		// Time out, no data returned
		return -1;
	}

	@Override
	public void i2cReturnData(SensorData data) {
		int[] rawData = (int[]) data.getData();
		i2cDataSize = rawData.length;
		for (int i = 0; i < i2cDataSize; i++) {
			i2cData[i] = (byte) (rawData[i] & 0xff);
		}
		log.debug("i2cReturnData invoked");
		i2cDataReturned = true;
	}

	@Override
	public void i2cWrite(I2CControl control, int busAddress, int deviceAddress, byte[] buffer, int size) {
		String i2cBus = String.format("I2CBus%s", busAddress);
		DeviceMapping deviceMapping = deviceList.get(i2cBus);
		int id = deviceMapping.getId();

		int data[] = new int[size];
		for (int i = 0; i < size; ++i) {
			data[i] = buffer[i];// guess you want -128 to 127 ?? [ ] == unsigned
								// char & 0xff;
		}

		msg.i2cWrite(id, deviceAddress, data);
	}

	@Override
	public int i2cWriteRead(I2CControl control, int busAddress, int deviceAddress, byte[] writeBuffer, int writeSize, byte[] readBuffer, int readSize) {
		if (writeSize != 1) {
			i2cWrite(control, busAddress, deviceAddress, writeBuffer, writeSize);
			return i2cRead(control, busAddress, deviceAddress, readBuffer, readSize);
		} else {
			i2cDataReturned = false;
			// Get the device index to the MRL i2c bus
			String i2cBus = String.format("I2CBus%s", busAddress);
			DeviceMapping map;
			map = deviceList.get(i2cBus);
			int id = map.getId(); // Device index to the I2CBus
			int msgBuffer[] = new int[4];
			msgBuffer[0] = id;
			msgBuffer[1] = deviceAddress;
			msgBuffer[2] = readSize;
			msgBuffer[3] = writeBuffer[0];
			msg.i2cWriteRead(getDeviceId(control), deviceAddress, readSize, writeBuffer[0] & 0xFF);
			int retry = 0;
			int retryMax = 1000; // ( About 1000ms = s)
			try {
				/**
				 * We will wait up to retryMax times to get the i2c data back
				 * from MRLComm.c and wait 1 ms between each try. A blocking
				 * queue is not needed, as this is only a single data element -
				 * and blocking is not necessary.
				 */
				while ((retry < retryMax) && (!i2cDataReturned)) {
					sleep(1);
					++retry;
				}
			} catch (Exception e) {
				Logging.logError(e);
			}
			if (i2cDataReturned) {
				log.debug(String.format("i2cReturnData returned %s bytes to caller %s.", i2cDataSize, control.getName()));
				for (int i = 0; i < i2cDataSize; i++) {
					readBuffer[i] = i2cData[i];
					log.debug(String.format("i2cReturnData returned ix %s value %s", i, readBuffer[i]));
				}
				return i2cDataSize;
			}
			// Time out, no data returned
			return -1;
		}
	}

	@Override
	public boolean isAttached() {
		return true;
	}

	public boolean isConnected() {
		// include that we must have gotten a valid MRLComm version number.
		if (serial != null && serial.isConnected() && boardInfo.getVersion() != null) {
			return true;
		}
		// FIXME - remove concept of rootController ..
		if (rootController != null && rootController.isConnected() && boardInfo.getVersion() != null) {
			return true;
		}
		return false;
	}

	@Override
	public boolean isRecording() {
		return record != null;
	}

	@Override
	public void motorMove(MotorControl mc) {

		MotorConfig c = mc.getConfig();

		if (c == null) {
			error("motor config not set");
			return;
		}

		Class<?> type = mc.getConfig().getClass();

		double powerOutput = mc.getPowerOutput();

		if (MotorConfigSimpleH.class == type) {
			MotorConfigSimpleH config = (MotorConfigSimpleH) c;
			msg.digitalWrite(config.getDirPin(), (powerOutput < 0) ? MOTOR_BACKWARD : MOTOR_FORWARD);
			msg.analogWrite(config.getPwrPin(), (int) Math.abs(powerOutput));
		} else if (MotorConfigDualPwm.class == type) {
			MotorConfigDualPwm config = (MotorConfigDualPwm) c;
			if (powerOutput < 0) {
				msg.analogWrite(config.getLeftPin(), 0);
				msg.analogWrite(config.getRightPin(), (int) Math.abs(powerOutput));
			} else if (powerOutput > 0) {
				msg.analogWrite(config.getRightPin(), 0);
				msg.analogWrite(config.getLeftPin(), (int) Math.abs(powerOutput));
			} else {
				msg.analogWrite(config.getLeftPin(), 0);
				msg.analogWrite(config.getRightPin(), 0);
			}
		} else if (MotorPulse.class == type) {
			MotorPulse motor = (MotorPulse) mc;
			// sdsendMsg(ANALOG_WRITE, motor.getPin(Motor.PIN_TYPE_PWM_RIGHT),
			// 0);
			// TODO implement with a -1 for "endless" pulses or a different
			// command parameter :P
			// sendMsg(new
			// MrlMsg(PULSE).append(motor.getPulsePin()).append((int)
			// Math.abs(powerOutput)));
		} else {
			error("motorMove for motor type %s not supported", type);
		}

	}

	// ========== pulsePin begin =============

	@Override
	public void motorMoveTo(MotorControl mc) {
		// speed parameter?
		// modulo - if < 1
		// speed = 1 else
		log.info("motorMoveTo targetPos {} powerLevel {}", mc.getTargetPos(), mc.getPowerLevel());

		Class<?> type = mc.getClass();

		// if pulser (with or without fake encoder
		// send a series of pulses !
		// with current direction
		if (MotorPulse.class == type) {
			MotorPulse motor = (MotorPulse) mc;
			// check motor direction
			// send motor direction
			// TODO powerLevel = 100 * powerlevel

			// FIXME !!! - this will have to send a Long for targetPos at some
			// point !!!!
			double target = Math.abs(motor.getTargetPos());

			int b0 = (int) target & 0xff;
			int b1 = ((int) target >> 8) & 0xff;
			int b2 = ((int) target >> 16) & 0xff;
			int b3 = ((int) target >> 24) & 0xff;

			// TODO FIXME
			// sendMsg(PULSE, deviceList.get(motor.getName()).id, b3, b2, b1,
			// b0, (int) motor.getPowerLevel(), feedbackRate);
		}

	}

	@Override
	public void motorReset(MotorControl motor) {
		// perhaps this should be in the motor control
		// motor.reset();
		// opportunity to reset variables on the controller
		// sendMsg(MOTOR_RESET, motor.getind);
	}

	@Override
	public void motorStop(MotorControl mc) {
		MotorConfig c = mc.getConfig();

		if (c == null) {
			error("motor config not set");
			return;
		}

		Class<?> type = mc.getConfig().getClass();

		if (MotorConfigPulse.class == type) {
			MotorConfigPulse config = (MotorConfigPulse) mc.getConfig();
			// sendMsg(new MrlMsg(PULSE_STOP).append(config.getPulsePin()));
		} else if (MotorConfigSimpleH.class == type) {
			MotorConfigSimpleH config = (MotorConfigSimpleH) mc.getConfig();
			msg.analogWrite(config.getPwrPin(), 0);
		} else if (MotorConfigDualPwm.class == type) {
			MotorConfigDualPwm config = (MotorConfigDualPwm) mc.getConfig();
			msg.analogWrite(config.getLeftPin(), 0);
			msg.analogWrite(config.getRightPin(), 0);
		}
	}

	@Override
	// > neoPixelAttach/deviceId/pin/b32 numPixels
	public void neoPixelAttach(NeoPixel neopixel, int pin, int numPixels) {
		msg.neoPixelAttach(getDeviceId(neopixel)/*byte*/, pin/*byte*/, numPixels/*b32*/);
	}

	@Override
	// > neoPixelSetAnimation/deviceId/animation/red/green/blue/b16 speed
	public void neoPixelSetAnimation(NeoPixel neopixel, int animation, int red, int green, int blue, int speed) {
		msg.neoPixelSetAnimation(getDeviceId(neopixel), animation, red, green, blue, speed);
	}

	@Override
	// > neoPixelWriteMatrix/deviceId/[] buffer
	public void neoPixelWriteMatrix(NeoPixel neopixel, List<Integer> data) {
		int id = getDeviceId(neopixel);
		int[] buffer = new int[data.size() + 2];
		buffer[0] = id;
		buffer[1] = data.size();
		for (int i = 0; i < data.size(); i++) {
			buffer[i + 2] = data.get(i);
		}
		// calamity, you can parameter the schema with id if you want ... size
		// travels with arrays
		// so you don't need to send size separately, different parameters can
		// make it more clear to read
		msg.neoPixelWriteMatrix(getDeviceId(neopixel), buffer);
	}

	/**
	 * Callback for Serial service - local (not remote) although a
	 * publish/subscribe could be created - this method is called by a thread
	 * waiting on the Serial's RX BlockingQueue
	 *
	 * Other services may use the same technique or subscribe to a Serial's
	 * publishByte method
	 *
	 * it might be worthwhile to look in optimizing reads into arrays vs single
	 * byte processing .. but maybe there would be no gain
	 *
	 */

	// FIXME - onByte(int[] data)
	@Override
	public Integer onByte(Integer newByte) {
		try {
			/**
			 * Archtype InputStream read - rxtxLib does not have this
			 * straightforward design, but the details of how it behaves is is
			 * handled in the Serial service and we are given a unified
			 * interface
			 *
			 * The "read()" is data taken from a blocking queue in the Serial
			 * service. If we want to support blocking functions in Arduino then
			 * we'll "publish" to our local queues
			 */
			// TODO: consider reading more than 1 byte at a time ,and make this
			// callback onBytes or something like that.

			++byteCount;
			if (log.isDebugEnabled()) {
				log.info("onByte {} \tbyteCount \t{}", newByte, byteCount);
			}
			if (byteCount == 1) {
				if (newByte != MAGIC_NUMBER) {
					byteCount = 0;
					msgSize = 0;
					Arrays.fill(ioCmd, 0); // FIXME - optimize - remove
					warn(String.format("Arduino->MRL error - bad magic number %d - %d rx errors", newByte, ++error_arduino_to_mrl_rx_cnt));
					// dump.setLength(0);
				}
				return newByte;
			} else if (byteCount == 2) {
				// get the size of message
				if (newByte > 64) {
					byteCount = 0;
					msgSize = 0;
					error(String.format("Arduino->MRL error %d rx sz errors", ++error_arduino_to_mrl_rx_cnt));
					return newByte;
				}
				msgSize = newByte.intValue();
				// dump.append(String.format("MSG|SZ %d", msgSize));
			} else if (byteCount > 2) {
				// remove header - fill msg data - (2) headbytes -1
				// (offset)
				// dump.append(String.format("|P%d %d", byteCount,
				// newByte));
				ioCmd[byteCount - 3] = newByte.intValue();
			} else {
				// the case where byteCount is negative?! not got.
				error(String.format("Arduino->MRL error %d rx negsz errors", ++error_arduino_to_mrl_rx_cnt));
				return newByte;
			}
			if (byteCount == 2 + msgSize) {
				// we've received a full message
				// process valid message
				// TODO: deserialize this byte array as an mrl message object to
				// help clean up the code.
				// int[] payload = Arrays.copyOfRange(msg, 2, msgSize);
				// MrlCommMessage mrlMsg = new MrlCommMessage(msg[0], payload);
				processMessage(ioCmd);
				// clean up memory/buffers
				msgSize = 0;
				byteCount = 0;
				Arrays.fill(ioCmd, 0); // optimize remove
			}
		} catch (Exception e) {
			++error_mrl_to_arduino_rx_cnt;
			error("msg structure violation %d", error_mrl_to_arduino_rx_cnt);
			log.warn("msg_structure violation byteCount {} buffer {}", byteCount, Arrays.copyOf(ioCmd, byteCount));
			// try again (clean up memory buffer)
			msgSize = 0;
			byteCount = 0;
			Logging.logError(e);
		}
		return newByte;
	}

	@Override
	public String onConnect(String portName) {
		info("%s connected to %s", getName(), portName);
		// Get version should already have been called. don't call it again!
		// getVersion();
		return portName;
	}

	public void onCustomMsg(Integer ax, Integer ay, Integer az) {
		log.info("onCustomMsg");
	}

	@Override
	public String onDisconnect(String portName) {
		info("%s disconnected from %s", getName(), portName);
		return portName;
	}

	/**
	 * Arduino is a sensor listener - it listens to its own pins and enabled Pin
	 * events
	 */
	@Override
	public void onSensorData(SensorData data) {

		// at the moment we do not need a 'type'
		// all sensor array data will be sent here ..

		// this is MRLComms raw data handled in the context of
		// a Sensor's onSensorData - so this method is specific for
		// transforming 'Arduino' specific data to a useful form
		// Since Arduino's useful data are pins we convert the pin values
		// read into an array of pins

		// we publish both pins and arrays of pins
		// it depends what other services have registered for

		// regardless we need to convert it all to PinData

		// it better be an array[] of ints with tuples in the form
		// | address | msb | lsb | ...
		int[] rawPinData = (int[]) data.getData();

		int pinDataCnt = rawPinData.length
				/ 3; /* length / (1 address + 1 msg + 1 lsb) */
		if (rawPinData.length % 3 != 0) {
			log.error("something is wrong - expecting 3 bytes per pin data");
		}

		PinData[] pinArray = new PinData[pinDataCnt];

		// parse sort reduce ...
		for (int i = 0; i < pinArray.length; ++i) {
			PinData pinData = new PinData(rawPinData[3 * i], Serial.bytesToInt(rawPinData, (3 * i) + 1, 2));
			pinArray[i] = pinData;
			int address = pinData.getAddress();

			// handle individual pins
			if (pinListeners.containsKey(address)) {
				List<PinListener> list = pinListeners.get(address);
				for (int j = 0; j < list.size(); ++j) {
					PinListener pinListner = list.get(j);
					if (pinListner.isLocal()) {
						pinListner.onPin(pinData);
					} else {
						invoke("publishPin", pinData);
					}
				}
			}
		}

		// publish array
		invoke("publishPinArray", new Object[] { pinArray });
	}

	public void pinMode(int pin, int mode) {
		msg.pinMode(pin, mode);
		PinDefinition pinDef = pinIndex.get(pin);
		invoke("publishPinDefinition", pinDef);
	}

	@Override
	public void pinMode(int address, String mode) {
		if (mode != null && mode.equalsIgnoreCase("INPUT")) {
			pinMode(address, INPUT);
		} else {
			pinMode(address, OUTPUT);
		}
	}

	/*
	 * public BoardInfo publishBoardInfo(BoardInfo info) { return info; }
	 */

	/**
	 * With Arduino we want to be able to do pinMode("D7", "INPUT"), but it
	 * should not be part of the PinArrayControl interface - because when it
	 * comes down to it .. a pin MUST ALWAYS have an address regardless what you
	 * label or name it...
	 * 
	 * @param pinName
	 * @param mode
	 */
	public void pinMode(String pinName, String mode) {
		if (mode != null && mode.equalsIgnoreCase("INPUT")) {
			pinMode(pinNameToAddress(pinName), mode);
		} else {
			pinMode(pinNameToAddress(pinName), mode);
		}
	}

	public Integer pinNameToAddress(String pinName) {
		if (!pinMap.containsKey(pinName)) {
			error("no pin %s exists", pinName);
			return null;
		}
		return pinMap.get(pinName).getAddress();
	}

	// This is called when a valid message is received from the serial port.
	private void processMessage(int[] message) {
		msg.processCommand(message);
	}

	public String publishAttachedDevice(int deviceId/* byte */, String deviceName/* str */) {

		if (record != null) {
			recordRxBuffer.append("/");
			recordRxBuffer.append(deviceId);
			recordRxBuffer.append("/");
			recordRxBuffer.append(deviceName);
		}

		if (!deviceList.containsKey(deviceName)) {
			error("PUBLISH_ATTACHED_DEVICE deviceName %s not found !", deviceName);
		}

		DeviceMapping deviceMapping = deviceList.get(deviceName);
		deviceMapping.setId(deviceId);
		deviceIndex.put(deviceId, deviceList.get(deviceName));

		// REMOVE
		invoke("publishAttachedDevice", deviceName);

		info("==== ATTACHED DEVICE %s WITH MRLDEVICE %d ====", deviceName, deviceId);

		return deviceName;
	}

	public BoardInfo publishBoardInfo(int version/* byte */, int boardType/* byte */) {
		boardInfo.setVersion(version);
		boardInfo.setType(boardType);

		// FIXME - have record part of generated code !!!
		if (record != null) {
			recordRxBuffer.append("/");
			recordRxBuffer.append(version);
			recordRxBuffer.append("/");
			recordRxBuffer.append(boardType);
		}

		log.info("Version return by Arduino: {}", boardInfo.getVersion());
		log.info("Board type returned by Arduino: {}", boardInfo.getName());
		log.info("Board type currently set: {}", boardType);
		if (!boardInfo.isUnknown()) {
			setBoard(boardInfo.getName());
			log.info("Board type set to: {}", boardType);
		} else {
			log.info("No change in board type");
		}

		invoke("publishBoardInfo", boardInfo);

		synchronized (boardInfo) {
			boardInfo.notifyAll();
		}

		return boardInfo;
	}

	public BoardStatus publishBoardStatus(BoardStatus status) {
		return status;
	}

	// public BoardInfo publishBoardInfo(int version/* byte */, int boardType/*
	// byte */) {
	public void publishBoardStatus(int b16i/* b16 */, int b32i/* b32 */, long bu32i/* bu32 */, String name2/* str */) {
		log.info(" testMsg deviceType {} name {} config {}", bu32i, name2);
		// return new BoardInfo(version, boardType);
	}

	public BoardStatus publishBoardStatus(Integer microsPerLoop/* b16 */, Integer sram/* b16 */, int[] deviceSummary/* byte */) {
		log.info("publishBoardStatus {} us, {} sram, {} devices", microsPerLoop, sram, deviceSummary);
		return new BoardStatus(microsPerLoop, sram, deviceSummary);
	}

	public int[] publishCustomMsg(int[] msg/* [] */) {
		return msg;
	}

	public String publishDebug(String debugMsg/* str */) {
		log.info("publishDebug {}", debugMsg);
		return debugMsg;
	}

	// public void publishEcho(String name1, int b8, long bui32, int bi32, int
	// b9, String name2, int[] config, long bui322) {
	// public void publishEcho(String name1/*str*/, int b8/*byte*/, Long
	// bui32/*bu32*/, int bi32/*b32*/, int b9/*byte*/, String name2/*str*/,
	// int[] config/*[]*/, Long bui322/*bu32*/){
	public void publishEcho(Long b32) {
		log.info("b32 {} ", b32);
	}

	public void publishMessageAck(int function/* byte */) {
	}

	// ============== GENERATED CALLBACKS BEGIN - non MrlObjects
	// ======================
	public String publishMRLCommError(String errorMsg/* str */) {
		log.error(errorMsg);
		return errorMsg;
	}

	/**
	 * This method is called with Pin data whene a pin value is changed on the
	 * Arduino board the Arduino must be told to poll the desired pin(s). This
	 * is done with a analogReadPollingStart(pin) or digitalReadPollingStart()
	 */

	public PinData publishPin(PinData pinData) {
		// caching last value
		pinIndex.get(pinData.getAddress()).setValue(pinData.getValue());
		return pinData;
	}

	/**
	 * publish all read pin data in one array at once
	 */
	public PinData[] publishPinArray(PinData[] pinData) {
		// FIXME - update all cache
		// pinIndex.get(pinEvent.getAddress()).setValue(pinEvent.getValue());
		return pinData;
	}

	/**
	 * method to communicate changes in pinmode or state changes
	 * 
	 * @param pinDef
	 * @return
	 */
	public PinDefinition publishPinDefinition(PinDefinition pinDef) {
		return pinDef;
	}

	public Long publishPulse(Long pulseCount) {
		return pulseCount;
	}

	/**
	 * published stop of a pulse series this occurs when count # of pulses has
	 * been reached or user intervention
	 *
	 * @param currentCount
	 * @return
	 */
	public Integer publishPulseStop(Integer currentCount) {
		return currentCount;
	}

	public void publishSensorData(int deviceId/* byte */, int[] data/* [] */) {
	}

	/**
	 * SensorDataPublisher.publishSensorData - publishes SensorData
	 */
	@Override
	public SensorData publishSensorData(SensorData data) {
		return data;
	}

	public void publishServoEvent(int deviceId/* byte */, int eventType/* byte */, int currentPos/* byte */, int targetPos/* byte */) {
	}

	public int publishServoEvent(Integer pos) {
		return pos;
	}

	public Pin publishTrigger(Pin pin) {
		return pin;
	}

	public Integer publishVersion(Integer version) {
		info("publishVersion %d", version);
		return version;
	}

	// PIN CONTROL //////////////////////////////////
	// FIXME implement
	// digitalRead for digital pins
	// analogRead for analog pins
	// block for callback
	@Override
	public int read(int address) {
		return pinIndex.get(address).getValue();
	}

	@Override
	public int read(String pinName) {
		return read(pinNameToAddress(pinName));
	}

	@Override
	public void record() throws Exception {
		if (record == null) {
			record = new FileOutputStream(String.format("%s.ard", getName()));
		}
	}

	public void refresh() {
		serial.refresh();
		broadcastState();
	}

	@Override
	public void releaseI2cDevice(I2CControl control, int busAddress, int deviceAddress) {
		// This method should delete the i2c device entry from the list of
		// I2CDevices
		String key = String.format("%d.%d", busAddress, deviceAddress);
		if (i2cDevices.containsKey(key)) {
			i2cDevices.remove(key);
		}
	}

	@Override
	public void releaseService() {
		super.releaseService();
		sleep(300);
		disconnect();
	}

	/**
	 * resets both MrlComm-land & Java-land
	 */
	public void reset() {
		log.info("reset - resetting all devices");

		// reset MrlComm-land
		softReset();

		for (String name : deviceList.keySet()) {
			DeviceMapping dmap = deviceList.get(name);
			DeviceControl device = dmap.getDevice();
			log.info("unsetting device {}", name);
			device.unsetController();
		}

		// reset Java-land
		deviceIndex.clear();
		deviceList.clear();
		error_mrl_to_arduino_rx_cnt = 0;
		error_arduino_to_mrl_rx_cnt = 0;
	}

	// FIXME -
	@Override
	public void sensorActivate(UltrasonicSensorControl sensor, Object... conf) {
		// TODO Auto-generated method stub

	}

	@Override
	public void sensorAttach(UltrasonicSensorControl sensor, int trigPin, int echoPin) {
		// TODO Auto-generated method stub

	}

	@Override
	public void sensorDeactivate(UltrasonicSensorControl sensor) {
		// TODO Auto-generated method stub

	}

	@Override
	public void servoAttach(ServoControl servo, int pin) {
		msg.servoEnablePwm(getDeviceId(servo), pin);
	}

	@Override
	public void servoAttach(ServoControl servo, int pin, Integer targetOutput, Integer velocity) {
		Integer deviceId = attachDevice(servo, new Object[] { pin, targetOutput, velocity });
		msg.servoAttach(deviceId, pin, targetOutput, velocity);
	}

	/**
	 * THESE ARE SERVO COMMANDS ! NOT REQUEST TO ATTACH OR DETACH THE SERVO AS A
	 * DEVICE !!! This is Servo.attach(10) .. not Arduino.attach(Device) !!
	 */
	@Override
	public void servoDetach(ServoControl servo) {
		msg.servoDisablePwm(getDeviceId(servo));
	}

	@Override
	public void servoSetMaxVelocity(ServoControl servo) {
		msg.servoSetMaxVelocity(getDeviceId(servo), servo.getMaxVelocity());
	}

	@Override
	public void servoSetVelocity(ServoControl servo) {
		msg.servoSetVelocity(getDeviceId(servo), servo.getVelocity());
	}

	@Override
	public void servoSweepStart(ServoControl servo) {
		int deviceId = getDeviceId(servo);
		log.info(String.format("servoSweep %s id %d min %d max %d step %d", servo.getName(), deviceId, servo.getSweepMin(), servo.getSweepMax(), servo.getSweepStep()));
		msg.servoSweepStart(deviceId, servo.getSweepMin(), servo.getSweepMax(), servo.getSweepStep());
	}

	@Override
	public void servoSweepStop(ServoControl servo) {
		msg.servoSweepStop(getDeviceId(servo));
	}

	@Override
	public void servoWrite(ServoControl servo) {
		int deviceId = getDeviceId(servo);
		log.info("servoWrite {} {} id {}", servo.getName(), servo.getTargetOutput(), deviceId);
		msg.servoWrite(deviceId, servo.getTargetOutput().intValue());
	}

	@Override
	public void servoWriteMicroseconds(ServoControl servo, int uS) {
		int deviceId = getDeviceId(servo);
		log.info(String.format("writeMicroseconds %s %d id %d", servo.getName(), uS, deviceId));
		msg.servoWriteMicroseconds(deviceId, uS);
	}

	public String setBoard(String board) {
		log.info("setting board to type {}", board);
		this.boardType = board;
		createPinList();
		broadcastState();
		return board;
	}

	/**
	 * easy way to set to a 54 pin arduino
	 *
	 * @return
	 */
	public String setBoardMega() {
		return setBoard(BOARD_TYPE_MEGA);
	}

	public String setBoardMegaADK() {
		return setBoard(BOARD_TYPE_MEGA_ADK);
	}

	public String setBoardUno() {
		return setBoard(BOARD_TYPE_UNO);
	}

	/**
	 * DeviceControl methods. In this case they represents the I2CBusControl Not
	 * sure if this is good to use the Arduino as an I2CBusControl Exploring
	 * different alternatives. I may have to rethink. Alternate solutions are
	 * welcome. /Mats.
	 */

	// TODO - probably be used by Arduino --controls--> other Arduino through
	// relay serial
	@Override
	public void setController(DeviceController controller) {
		// TODO Auto-generated method stub
		// Not sure what to do here. I don't want to create an infinite loop
	}

	/**
	 * Debounce ensures that only a single signal will be acted upon for a
	 * single opening or closing of a contact. the delay is the min number of pc
	 * cycles must occur before a reading is taken
	 *
	 * Affects all reading of pins setting to 0 sets it off
	 *
	 * @param delay
	 */
	public void setDebounce(int pin, int delay) {
		msg.setDebounce(pin, delay);
	}

	public void setDebug(boolean b) {
		msg.setDebug(b);
	}

	public void setSerialRate(int rate) {
		msg.setSerialRate(rate);
	}

	public void setSketch(Sketch sketch) {
		this.sketch = sketch;
		broadcastState();
	}

	/**
	 * set a pin trigger where a value will be sampled and an event will be
	 * signal when the pin turns into a different state.
	 * 
	 * TODO - implement on MrlComm side...
	 *
	 * @param pin
	 * @param value
	 * @param type
	 * @return
	 */
	public void setTrigger(int pin, int value) {
		msg.setTrigger(pin, value);
	}

	/**
	 * send a reset to MrlComm - all devices removed, all polling is stopped and
	 * all other counters are reset
	 *
	 * TODO - reset servos ? motors ? etc. ?
	 */
	public void softReset() {
		msg.softReset();
	}

	@Override
	public void startService() {
		super.startService();
		try {
			serial = (Serial) startPeer("serial");
			msg = new Msg(this, serial);
			// FIXME - dynamically additive - if codec key has never been used -
			// add key
			// serial.getOutbox().setBlocking(true);
			// inbox.setBlocking(true);
			serial.addByteListener(this);
		} catch (Exception e) {
			Logging.logError(e);
		}
	}

	@Override
	public void stopRecording() {
		if (record != null) {
			try {
				record.close();
			} catch (Exception e) {
			}
			record = null;
		}
	}

	@Override
	public void stopService() {
		super.stopService();
		disconnect();
	}

	public void test() {
		int[] config = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
		for (int i = 0; i < 10000; ++i) {
			// msg.echo("name1", 8, 999999999, 121212, 254, "name2", config,
			// 5454545);
			// b32 long int 4 bytes -2,147,483,648 to 2,147,483, 647
			// bu32 unsigned long long 0 to 4,294,967,295

			// 2147483647
			// 2139062143
			// msg.echo(2147483647, "hello", 33, 2147483647L, 32767, 25, "oink
			// oink", config, 2147483647L);
			msg.echo(4294967296L);
			// msg.echo(32767, "hello 1", 127, 2147418111, 32767, 8, "name 2 is
			// here", config, 534332);
			// 2147418111
			// 2147352575
		}
	}

	@Override
	public void unsetController() {

	}

	public void uploadSketch(String arduinoPath) throws IOException {
		uploadSketch(arduinoPath, serial.getLastPortName());
	}

	public void uploadSketch(String arudinoPath, String comPort) throws IOException {
		uploadSketch(arudinoPath, comPort, getBoardType());
	}

	public void uploadSketch(String arduinoIdePath, String port, String type) throws IOException {
		log.info("uploadSketch ({}, {}, {})", arduinoIdePath, port, type);
		// hail mary - if we have no idea
		// guess uno
		if (type == null) {
			type = BOARD_TYPE_UNO;
		}

		arduinoIdePath = arduinoIdePath.replace("\\", "/");
		arduinoIdePath = arduinoIdePath.trim();
		if (!arduinoIdePath.endsWith("/")) {
			arduinoIdePath += "/";
		}

		log.info(String.format("arduino IDE Path=%s", arduinoIdePath));
		log.info(String.format("Port=%s", port));
		log.info(String.format("type=%s", type));
		if (arduinoIdePath != null && !arduinoIdePath.equals(ArduinoUtils.arduinoPath)) {
			this.arduinoPath = arduinoIdePath;
			ArduinoUtils.arduinoPath = arduinoIdePath;
			save();
		}

		uploadSketchResult = String.format("Uploaded %s ", new Date());

		boolean connectedState = isConnected();
		try {

			if (connectedState) {
				log.info("disconnecting...");
				disconnect();
			}
			ArduinoUtils.uploadSketch(port, type.toLowerCase());

		} catch (Exception e) {
			log.info("ArduinoUtils threw trying to upload", e);
		}

		if (connectedState) {
			log.info("reconnecting...");
			serial.connect();
		}

		// perhaps you can reduce the inter-process information
		// to succeed | fail .. perhaps you can't
		// I would prefer transparency - send all output to the ui
		uploadSketchResult += ArduinoUtils.getOutput();

		log.info(uploadSketchResult);
		broadcastState();
	}

	/**
	 * PinArrayControl method
	 */
	@Override
	public void write(int address, int value) {
		info("write (%d,%d) to %s", address, value, serial.getName());

		PinDefinition pinDef = pinIndex.get(address);

		if (pinDef.isPwm()) {
			analogWrite(address, value);
		} else {
			digitalWrite(address, value);
		}
		// cache value
		pinDef.setValue(value);
	}

	// ============== GENERATED CALLBACKS END - non MrlObjects
	// ======================

}
