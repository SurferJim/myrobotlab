package org.myrobotlab.service;

import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MAGIC_NUMBER;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MAX_MSG_SIZE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MSG_ROUTE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_BOARD_INFO;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_BOARD_STATUS;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_MRLCOMM_ERROR;

import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MAX_MSG_SIZE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MAGIC_NUMBER;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MRLCOMM_VERSION;

import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_NOT_FOUND;

import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_ARDUINO;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_ULTRASONIC;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_STEPPER;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_MOTOR;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_STEPPER;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_MOTOR;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_SERVO;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_I2C;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_TYPE_NEOPIXEL;

///// java static import definition - DO NOT MODIFY - Begin //////
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_MRLCOMM_ERROR;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.GET_BOARD_INFO;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_BOARD_INFO;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.ANALOG_WRITE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.CONTROLLER_ATTACH;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.CREATE_I2C_DEVICE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.CUSTOM_MSG;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_ATTACH;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DEVICE_DETACH;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DIGITAL_WRITE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DISABLE_BOARD_STATUS;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DISABLE_PIN;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.DISABLE_PINS;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.ENABLE_BOARD_STATUS;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.ENABLE_PIN;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.GET_MRL_PIN_TYPE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.HEARTBEAT;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.I2C_READ;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.I2C_RETURN_DATA;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.I2C_WRITE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.I2C_WRITE_READ;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.INTS_TO_STRING;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.IS_ATTACHED;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.IS_RECORDING;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MOTOR_MOVE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MOTOR_MOVE_TO;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MOTOR_RESET;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MOTOR_STOP;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.MSG_ROUTE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.NEO_PIXEL_SET_ANIMATION;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.NEO_PIXEL_WRITE_MATRIX;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.ON_SENSOR_DATA;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PIN_MODE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_ATTACHED_DEVICE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_BOARD_STATUS;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_CUSTOM_MSG;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_DEBUG;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_MESSAGE_ACK;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_PIN;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_PIN_ARRAY;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_PIN_DEFINITION;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_PULSE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_PULSE_STOP;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_SENSOR_DATA;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_SERVO_EVENT;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PUBLISH_TRIGGER;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PULSE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.PULSE_STOP;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.READ;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.RECORD;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.RELEASE_I2C_DEVICE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.RESET;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SENSOR_ACTIVATE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SENSOR_DEACTIVATE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SENSOR_POLLING_START;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SENSOR_POLLING_STOP;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SERVO_ATTACH;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SERVO_DETACH;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SERVO_SET_MAX_VELOCITY;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SERVO_SET_VELOCITY;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SERVO_SWEEP_START;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SERVO_SWEEP_STOP;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SERVO_WRITE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SERVO_WRITE_MICROSECONDS;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SET_BOARD_MEGA_ADK;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SET_DEBOUNCE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SET_DEBUG;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SET_DIGITAL_TRIGGER_ONLY;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SET_SERIAL_RATE;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SET_TRIGGER;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.SOFT_RESET;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.STOP_RECORDING;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.UNSET_CONTROLLER;
import static org.myrobotlab.codec.serial.ArduinoMsgCodec.WRITE;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import org.myrobotlab.arduino.BoardInfo;
import org.myrobotlab.arduino.BoardStatus;
import org.myrobotlab.arduino.MrlMsg;
import org.myrobotlab.codec.serial.ArduinoMsgCodec;
import org.myrobotlab.framework.Service;
import org.myrobotlab.framework.ServiceType;
import org.myrobotlab.logging.Level;
import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.logging.Logging;
import org.myrobotlab.logging.LoggingFactory;
import org.myrobotlab.serial.PortQueue;
import org.myrobotlab.service.interfaces.RecordControl;
import org.myrobotlab.service.interfaces.SerialDataListener;
import org.myrobotlab.service.interfaces.SerialDevice;
import org.slf4j.Logger;

public class VirtualArduino extends Service implements SerialDataListener, RecordControl {

	private static final long serialVersionUID = 1L;

	public final static Logger log = LoggerFactory.getLogger(VirtualArduino.class);

	Serial uart;
	String portName = "COM42";
	final BoardInfo boardInfo = new BoardInfo();
	String boardType;

	transient FileOutputStream record = null;
	// for debuging & developing - need synchronized - both send & recv threads
	transient StringBuffer recordRxBuffer = new StringBuffer();
	transient StringBuffer recordTxBuffer = new StringBuffer();

	private int byteCount;

	private int msgSize;

	public VirtualArduino(String n) {
		super(n);
		uart = (Serial) createPeer("uart");
		boardInfo.setVersion(MRLCOMM_VERSION);
	}

	@Override
	public void startService() {
		super.startService();
		try {
			uart = (Serial) startPeer("uart");
			connectVirtualUart(portName, portName + ".UART");
			uart.addByteListener(this);
			
			// since we don't have a real DTR CDR lines like
			// the Arduino
			// load 3 publishBoards on serial line
			
		} catch (Exception e) {
			log.error("starting service threw", e);
		}
	}

	public void setPortName(String portName) {
		this.portName = portName;
	}

	/**
	 * This static method returns all the details of the class without it having
	 * to be constructed. It has description, categories, dependencies, and peer
	 * definitions.
	 * 
	 * @return ServiceType - returns all the data
	 * 
	 */
	static public ServiceType getMetaData() {

		ServiceType meta = new ServiceType(VirtualArduino.class.getCanonicalName());
		meta.addDescription("virtual hardware of for the Arduino!");
		meta.setAvailable(true); // false if you do not want it viewable in a
									// gui
		meta.addPeer("uart", "Serial", "serial device for this Arduino");
		meta.addCategory("simulator");
		return meta;
	}

	public SerialDevice connectVirtualUart(String myPort, String uartPort) throws IOException {

		BlockingQueue<Integer> left = new LinkedBlockingQueue<Integer>();
		BlockingQueue<Integer> right = new LinkedBlockingQueue<Integer>();

		// add our virtual port
		PortQueue vPort = new PortQueue(myPort, left, right);
		Serial.ports.put(myPort, vPort);

		PortQueue uPort = new PortQueue(uartPort, right, left);
		uart.connectPort(uPort, uart);

		log.info(String.format("connectToVirtualUart - creating uart %s <--> %s", myPort, uartPort));
		return uart;
	}

	transient int[] msg = new int[MAX_MSG_SIZE];

	private int error_mrl_to_arduino_rx_cnt;

	@Override
	public Integer onByte(Integer newByte) {
		try {

			++byteCount;
			if (log.isDebugEnabled()) {
				log.info("onByte {} \tbyteCount \t{}", newByte, byteCount);
			}
			if (byteCount == 1) {
				if (newByte != MAGIC_NUMBER) {
					byteCount = 0;
					msgSize = 0;
					Arrays.fill(msg, MAGIC_NUMBER);
					warn(String.format("Arduino->MRL error - bad magic number %d - %d rx errors", newByte, ++error_mrl_to_arduino_rx_cnt));
					// dump.setLength(0);
				}
				return newByte;
			} else if (byteCount == 2) {
				// get the size of message
				if (newByte > 64) {
					byteCount = 0;
					msgSize = 0;
					error(String.format("Arduino->MRL error %d rx sz errors", ++error_mrl_to_arduino_rx_cnt));
					return newByte;
				}
				msgSize = (byte) newByte.intValue();
				// dump.append(String.format("MSG|SZ %d", msgSize));
			} else if (byteCount > 2) {
				// remove header - fill msg data - (2) headbytes -1
				// (offset)
				// dump.append(String.format("|P%d %d", byteCount,
				// newByte));
				msg[byteCount - 3] = (byte) newByte.intValue();
			} else {
				// the case where byteCount is negative?! not got.
				error(String.format("Arduino->MRL error %d rx negsz errors", ++error_mrl_to_arduino_rx_cnt));
				return newByte;
			}
			if (byteCount == 2 + msgSize) {
				// we've received a full message
				// process valid message
				// TODO: deserialize this byte array as an mrl message object to
				// help clean up the code.
				// int[] payload = Arrays.copyOfRange(msg, 2, msgSize);
				// MrlCommMessage mrlMsg = new MrlCommMessage(msg[0], payload);
				processMessage(msg);
				// clean up memory/buffers
				msgSize = 0;
				byteCount = 0;
				Arrays.fill(msg, 0); // optimize remove
			}
		} catch (Exception e) {
			++error_mrl_to_arduino_rx_cnt;
			error("msg structure violation %d", error_mrl_to_arduino_rx_cnt);
			log.warn("msg_structure violation byteCount {} buffer {}", byteCount, Arrays.copyOf(msg, byteCount));
			// try again (clean up memory buffer)
			msgSize = 0;
			byteCount = 0;
			Logging.logError(e);
		}
		return newByte;
	}

	public void processMessage(int[] message) {
		// MSG CONTENTS = FN | D0 | D1 | ...
		int function = message[0];
		// log.info("Process Message Called: {}",
		// ArduinoMsgCodec.functionToString(function));
		// if (log.isDebugEnabled()) {
		// log.debug("Process Message Called: {}",
		// ArduinoMsgCodec.functionToString(function));
		// }

		if (record != null) {
			recordRxBuffer.append("< ");
			recordRxBuffer.append(ArduinoMsgCodec.byteToMethod(function));
		}

		switch (function) {
		case GET_BOARD_INFO: {
			MrlMsg msg = new MrlMsg(PUBLISH_BOARD_INFO);
			msg.append(boardInfo.getVersion()).append(boardInfo.getBoardType());
			sendMsg(msg);
			break;
		}
		default: {
			// FIXME - use formatter for message
			error("not handled serial event %d - %s ?", function, ArduinoMsgCodec.byteToMethod(function));
			break;
		}
		}

		if (record != null) {

			// FIXME - hmm if it was not static - the angular client could use
			// it
			// FIXME - each callback would have to decode appropriately and
			// append to
			// a string buffer

			try {
				recordRxBuffer.append("\n");
				record.write(recordRxBuffer.toString().getBytes());
			} catch (IOException e) {
				log.error("recording threw", e);
			}

			recordRxBuffer.setLength(0);
		}

	}

	@Override
	public String onConnect(String portName) {
		for (int i = 0; i < 3; ++i) {
			MrlMsg msg = new MrlMsg(PUBLISH_BOARD_INFO);
			msg.append(boardInfo.getVersion()).append(boardInfo.getBoardType());
			sendMsg(msg);
		}
		return portName;
	}

	@Override
	public String onDisconnect(String portName) {
		return portName;
	}

	public synchronized boolean sendMsg(MrlMsg msg) {
		int function = msg.getMethod();

		List<Integer> list = msg.getList();
		int[] params = new int[list.size()];
		for (int i = 0; i < list.size(); ++i) {
			params[i] = list.get(i);
		}

		log.info(ArduinoMsgCodec.byteToMethod(function));
		if (record != null) {
			recordTxBuffer.append("> ");
			recordTxBuffer.append(msg.toString());
			recordTxBuffer.append("\n");
			try {
				record.write(recordTxBuffer.toString().getBytes());
			} catch (Exception e) {
			}
			recordTxBuffer.setLength(0);
		}

		// some sanity checking.
		if (!uart.isConnected()) {
			log.warn("Serial port is not connected, unable to send message.");
			return false;
		}
		// don't even attempt to send it if we know it's a bogus message.
		// TODO: we need to account for the magic byte & length bytes. max
		// message size is 64-2 (potentially)
		if (params.length > MAX_MSG_SIZE) {
			log.error("Arduino Message size was large! Function {} Size {}", function, params.length);
			return false;
		}
		// System.out.println("Sending Message " + function );
		try {// FIXME - perhaps we should
				// Minimum MRLComm message is 3 bytes(int).
				// MAGIC_NUMBER|LENGTH|FUNCTION|PARAM0|PARAM1 would be valid
			int[] msgToSend = new int[3 + params.length];
			msgToSend[0] = MAGIC_NUMBER;
			msgToSend[1] = 1 + params.length;
			msgToSend[2] = function;
			for (int i = 0; i < params.length; i++) {
				// What if the int is > 127 ?
				msgToSend[3 + i] = params[i];
			}
			// send the message as an array. (uart port actually writes 1
			// byte
			// at a time anyway.. oh well.)
			// notify();

			// Technically this is the only thing that needs to be
			// synchronized
			// i think.
			synchronized (msgToSend) {
				log.info("writing....");
				uart.write(msgToSend);
			}
		} catch (Exception e) {
			error("sendMsg " + e.getMessage());
			return false;
		}

		return true;
	}

	@Override
	public void record() throws Exception {
		if (record == null) {
			record = new FileOutputStream(String.format("%s.ard", getName()));
		}
		MrlMsg.enableText();
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
	public boolean isRecording() {
		return record != null;
	}

	public String setBoard(String board) {
		log.info("setting board to type {}", board);

		boardInfo.setType(board);
		this.boardType = board;
		// createPinList();
		broadcastState();
		return board;
	}

	/**
	 * easy way to set to a 54 pin arduino
	 *
	 * @return
	 */
	public String setBoardMega() {
		return setBoard(Arduino.BOARD_TYPE_MEGA);
	}

	public String setBoardMegaADK() {
		return setBoard(Arduino.BOARD_TYPE_MEGA_ADK);
	}

	public String setBoardUno() {
		return setBoard(Arduino.BOARD_TYPE_UNO);
	}

	public static void main(String[] args) {
		try {

			LoggingFactory.init(Level.INFO);

			VirtualArduino varduino = null;

			String port = "COM42";
			boolean useVirtual = true;

			Runtime.start("gui", "GUIService");
			Runtime.start("python", "Python");
			Arduino arduino = (Arduino) Runtime.start("arduino", "Arduino");
			arduino.record();
			log.info("ports " + Arrays.toString(arduino.getSerial().getPortNames().toArray()));
			if (useVirtual) {
				varduino = (VirtualArduino) Runtime.create("varduino", "VirtualArduino");
				varduino.setPortName(port);
				Runtime.start("varduino", "VirtualArduino");
				varduino.setBoardMega();//.setBoardUno();
			}
			arduino.connect(port);

		} catch (Exception e) {
			log.error("main threw", e);
		}
	}

}
