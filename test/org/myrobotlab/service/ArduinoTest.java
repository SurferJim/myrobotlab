package org.myrobotlab.service;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.List;

import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import org.junit.runner.Description;
import org.junit.runner.JUnitCore;
import org.junit.runner.Result;
import org.junit.runner.notification.Failure;
import org.junit.runner.notification.RunListener;
import org.myrobotlab.arduino.Msg;
import org.myrobotlab.logging.Level;
import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.logging.Logging;
import org.myrobotlab.logging.LoggingFactory;
import org.myrobotlab.service.Arduino.Sketch;
import org.myrobotlab.service.data.PinData;
import org.myrobotlab.service.interfaces.PinArrayListener;
import org.myrobotlab.service.interfaces.PinDefinition;
import org.slf4j.Logger;

/**
 * 
 * @author GroG
 *
 */

public class ArduinoTest implements PinArrayListener {

	public final static Logger log = LoggerFactory.getLogger(ArduinoTest.class);

	static boolean userVirtualHardware = true;

	static String port = "COM4";

	static Arduino arduino = null;
	static Serial serial = null;

	static VirtualArduino virtual = null;
	static Python logic = null;
	static Serial uart = null;

	int servoPin = 6;
	int enablePin = 15;

	// FIXME - test for re-entrant !!!!
	// FIXME - single switch for virtual versus "real" hardware

	@BeforeClass
	public static void setUpBeforeClass() throws Exception {
		log.info("setUpBeforeClass");

		arduino = (Arduino) Runtime.start("arduino", "Arduino");
		serial = arduino.getSerial();

		// FIXME - needs a seemless switch
		if (userVirtualHardware) {

			virtual = (VirtualArduino) Runtime.start("virtual", "VirtualArduino");
			uart = virtual.getSerial();
			uart.setTimeout(100); // don't want to hang when decoding results...

		}

		// arduino.connect(port);

	}

	@AfterClass
	public static void tearDownAfterClass() throws Exception {
	}

	@Before
	public void setUp() throws Exception {

		// expected state of arduino is no devices ?
		// & connected ?
		// arduino.connect(port);

		serial.clear();
		serial.setTimeout(100);

		uart.clear();
		uart.setTimeout(100);
	}

	@Test
	public void testReleaseService() {
		// fail("Not yet implemented");
	}

	@Test
	public void testStartService() {
		// fail("Not yet implemented");
	}

	@Test
	public void testStopService() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetMetaData() {
		// fail("Not yet implemented");
	}

	@Test
	public void testArduino() {
		// fail("Not yet implemented");
	}

	@Test
	public final void testAnalogWrite() throws InterruptedException, IOException {
		log.info("testAnalogWrite");

		uart.startRecording();
		arduino.analogWrite(10, 0);
		// assertEquals("analogWrite/10/0\n", decoded);

		arduino.analogWrite(10, 127);

		// assertEquals("analogWrite/10/127\n", decoded);

		arduino.analogWrite(10, 128);
		// assertEquals("analogWrite/10/128\n", decoded);

		arduino.analogWrite(10, 255);
		// assertEquals("analogWrite/10/255\n", decoded);

		arduino.error("test");
	}

	@Test
	public void testAttachPinArrayListener() {
		// fail("Not yet implemented");
	}

	@Test
	public void testAttachPinListenerInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testAttachStringInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testConnectArduinoString() {
		// fail("Not yet implemented");
	}

	@Test
	public void testConnectString() {
		for (int i = 0; i < 20; ++i) {
			arduino.connect(port);
			// arduino.enableAck(true);
			arduino.echo(30003030L + i);
			arduino.echo(2L);
			arduino.echo(-1L);
			arduino.disconnect();
		}

	}

	@Test
	public void testConnectStringIntIntIntInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testControllerAttach() {
		// fail("Not yet implemented");
	}

	@Test
	public void testCreatePinList() {
		// fail("Not yet implemented");
	}

	@Test
	public void testCustomMsg() {
		// fail("Not yet implemented");
	}

	@Test
	public void testDeviceDetach() {
		// fail("Not yet implemented");
	}

	@Test
	public final void testDigitalWrite() {
		log.info("testDigitalWrite");
		arduino.digitalWrite(10, 1);
		// assertEquals("digitalWrite/10/1\n", uart.decode());
		arduino.digitalWrite(10, 0);
		// assertEquals("digitalWrite/10/0\n", uart.decode());
		// arduino.digitalWrite(10, 255);
		// assertEquals("digitalWrite/10/0", uart.decode());
	}

	@Test
	public void testDisablePin() {
		// fail("Not yet implemented");
	}

	@Test
	public void testDisablePins() {
		// fail("Not yet implemented");
	}

	@Test
	public final void testDisconnect() throws IOException {
		log.info("testDisconnect");
		arduino.disconnect();
		assertTrue(!arduino.isConnected());
		arduino.digitalWrite(10, 1);
		assertEquals(0, uart.available());
		arduino.connect(port);
		assertTrue(arduino.isConnected());
		uart.clear();
		arduino.digitalWrite(10, 1);
		// assertEquals("digitalWrite/10/1\n", uart.decode());
	}

	@Test
	public void testEnableBoardStatus() {

		org.myrobotlab.service.Test test = (org.myrobotlab.service.Test) Runtime.start("test", "Test");
		test.subscribe(arduino.getName(), "publishBoardStatus");
		arduino.enableBoardStatus(true);
		// FIXME notify with timeout

		arduino.enableBoardStatus(false);
	}

	@Test
	public void testEnableBoardStatusInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testEnabledHeartbeat() {
		// fail("Not yet implemented");
	}

	@Test
	public void testEnablePinInt() {
		// set board type
		arduino.connect(port);
		arduino.enablePin(enablePin);
		arduino.attach(this);
	}

	@Test
	public void testEnablePinIntInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetBoardInfo() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetController() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetDeviceIdDeviceControl() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetDeviceIdString() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetMrlPinType() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetPinList() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetPortName() {
		// fail("Not yet implemented");
	}

	@Test
	public void testGetSerial() {
		// fail("Not yet implemented");
	}

	@Test
	public void testHeartbeat() {
		// fail("Not yet implemented");
	}

	@Test
	public void testI2cRead() {
		// fail("Not yet implemented");
	}

	@Test
	public void testI2cReturnData() {
		// fail("Not yet implemented");
	}

	@Test
	public void testI2cWrite() {
		// fail("Not yet implemented");
	}

	@Test
	public void testI2cWriteRead() {
		// fail("Not yet implemented");
	}

	@Test
	public void testIsAttached() {
		// fail("Not yet implemented");
	}

	@Test
	public void testIsConnected() {
		// fail("Not yet implemented");
	}

	@Test
	public void testIsRecording() {
		// fail("Not yet implemented");
	}

	@Test
	public void testMotorMove() {
		// fail("Not yet implemented");
	}

	@Test
	public void testMotorMoveTo() {
		// fail("Not yet implemented");
	}

	@Test
	public void testMotorReset() {
		// fail("Not yet implemented");
	}

	@Test
	public void testMotorStop() {
		// fail("Not yet implemented");
	}

	@Test
	public void testMsgRoute() {
		// fail("Not yet implemented");
	}

	@Test
	public void testNeoPixelSetAnimation() {
		// fail("Not yet implemented");
	}

	@Test
	public void testNeoPixelWriteMatrix() {
		// fail("Not yet implemented");
	}

	@Test
	public void testOnByte() {
		// fail("Not yet implemented");
	}

	@Test
	public void testOnConnect() {
		// fail("Not yet implemented");
	}

	@Test
	public void testOnCustomMsg() {
		// fail("Not yet implemented");
	}

	@Test
	public void testOnDisconnect() {
		// fail("Not yet implemented");
	}

	@Test
	public void testOnSensorData() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPinModeIntInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPinModeStringString() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPinNameToAddress() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishAttachedDevice() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishBoardInfo() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishBoardStatus() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishCustomMsg() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishDebug() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishMessageAck() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishMRLCommError() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishPin() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishPinArray() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishPinDefinition() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishPulse() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishPulseStop() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishSensorData() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishServoEvent() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishTrigger() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPublishVersion() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPulseInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPulseIntInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPulseIntIntInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPulseIntIntIntInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testPulseStop() {
		// fail("Not yet implemented");
	}

	@Test
	public void testReadInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testReadString() {
		// fail("Not yet implemented");
	}

	@Test
	public void testRecord() {
		// fail("Not yet implemented");
	}

	@Test
	public void testRefresh() {
		// fail("Not yet implemented");
	}

	@Test
	public void testReleaseI2cDevice() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSendMsgIntIntArray() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSendMsgIntListOfInteger() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSendMsgMrlMsg() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSensorActivate() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSensorDeactivate() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSensorPollingStart() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSensorPollingStop() {
		// fail("Not yet implemented");
	}

	@Test
	public void testServoAttach() {
		// fail("Not yet implemented");
	}

	@Test
	public void testServoDetach() {
		// fail("Not yet implemented");
	}

	@Test
	public void testServoSetMaxVelocity() {
		// fail("Not yet implemented");
	}

	@Test
	public void testServoSetVelocity() {
		// fail("Not yet implemented");
	}

	@Test
	public void testServoSweepStart() {
		// fail("Not yet implemented");
	}

	@Test
	public void testServoSweepStop() {
		// fail("Not yet implemented");
	}

	@Test
	public void testServoWrite() {
		// fail("Not yet implemented");
	}

	@Test
	public void testServoWriteMicroseconds() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetBoard() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetBoardMegaADK() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetController() {
		// fail("Not yet implemented");
	}

	@Test
	public void testUnsetController() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetDebounce() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetDebug() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetDigitalTriggerOnly() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetPWMFrequency() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetSerialRate() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetSketch() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetTriggerIntInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSetTriggerIntIntInt() {
		// fail("Not yet implemented");
	}

	@Test
	public void testSoftReset() {
		// fail("Not yet implemented");
	}

	@Test
	public void testReset() {
		// fail("Not yet implemented");
	}

	@Test
	public void testStopRecording() {
		// fail("Not yet implemented");
	}

	@Test
	public void testUploadSketchString() {
		// fail("Not yet implemented");
	}

	@Test
	public void testUploadSketchStringString() {
		// fail("Not yet implemented");
	}

	@Test
	public void testUploadSketchStringStringString() {
		// fail("Not yet implemented");
	}

	@Test
	public void testWrite() {
		// fail("Not yet implemented");
	}

	//////// end generated ///////////////////////

	@Test
	public final void testConnect() throws IOException {
		log.info("testConnect - begin");
		arduino.disconnect();
		arduino.connect(port);
		assertTrue(arduino.isConnected());
		assertEquals(Msg.MRLCOMM_VERSION, arduino.getBoardInfo().getVersion().intValue());
		log.info("testConnect - end");
	}

	@Test
	public void testGetBoardType() {
		// arduino.setBoardMega()
	}

	@Test
	public final void testGetSketch() {
		log.info("testGetSketch");
		Sketch sketch = arduino.getSketch();
		assertNotNull(sketch);
		assertTrue(sketch.data.length() > 0);
		arduino.setSketch(null);
		assertNull(arduino.getSketch());
		arduino.setSketch(sketch);
		assertEquals(sketch, arduino.getSketch());
	}

	@Test
	public final void testGetVersion() {
		log.info("testGetVersion");
		assertEquals(Msg.MRLCOMM_VERSION, arduino.getBoardInfo().getVersion().intValue());
	}

	@Test
	public final void testPinModeIntString() {
		log.info("testPinModeIntString");
		arduino.pinMode(8, "OUTPUT");
		// assertEquals("pinMode/8/1\n", uart.decode());
	}

	@Test
	public final void testPinModeIntegerInteger() {
		log.info("testPinModeIntegerInteger");
		arduino.pinMode(8, Arduino.OUTPUT);
		// assertEquals("pinMode/8/1\n", uart.decode());
	}

	@Test
	public final void testServoAttachServoInteger() throws Exception {
		log.info("testServoAttachServoInteger");
		Servo servo = null;
		Runtime.start("webgui", "WebGui");
		for (int i = 0; i < 3; ++i) {
			servo = (Servo) Runtime.start("servo", "Servo");
			arduino.connect(port);
			// NOT THE WAY TO ATTACH SERVOS !!
			// isAttached will not get set
			// dont know a good fix - asside from not using it !
			// arduino.servoAttach(servo, servoPin);
			// re-entrant test
			// arduino.servoAttach(servo, servoPin);

			// common way
			// arduino.servoAttach(servo, servoPin);
			// servo.attach(); // TEST ME WITHOUT PREVIOUS
			servo.attach(arduino, servoPin);

			// another way
			// servo.setPin(servoPin);
			// servo.setController(arduino);
			
			servo.setPin(servoPin);
			arduino.attach(servo);

			assertTrue(servo.isAttached());

			int velocity = 50;
			// degree per second
			servo.setVelocity(velocity);
			// assertEquals(virtual.servoSetVelocity(velocity));
			
			servo.attach(7);
			servo.moveTo(30);
			servo.moveTo(130);

			servo.attach(servoPin);
			
			servo.moveTo(130);
			// assertEquals(virtual.servoMoveTo(130));
			servo.moveTo(30);

			assertTrue(servo.isAttached());
			assertEquals(arduino.getName(), servo.getController().getName());

			servo.moveTo(0);
			// assertEquals(virtual.servoMoveTo(0));
			servo.moveTo(90);
			// assertEquals("servoWrite/7/90\n", uart.decode());
			servo.moveTo(180);
			// assertEquals("servoWrite/7/180\n", uart.decode());
			servo.moveTo(0);
			// assertEquals("servoWrite/7/0\n", uart.decode());

			// detach
			servo.detach();
			// assertEquals("servoDetach/7/0\n", uart.decode());

			servo.moveTo(10);

			// re-attach
			servo.attach();
			// assertEquals("servoAttach/7/9/5/115/101/114/118/111\n",
			// uart.decode());
			assertTrue(servo.isAttached());
			// // assertEquals(servoPin, servo.getPin().intValue());
			assertEquals(arduino.getName(), servo.getController().getName());

			servo.moveTo(90);
			// assertEquals("servoWrite/7/90\n", uart.decode());

			arduino.enableBoardStatus(true);

			servo.startService();

			servo.moveTo(90);
		}

		servo.releaseService();
		arduino.enableBoardStatus(false);
	}

	@Test
	public void testSetBoardMega() {
		log.info("testSetBoardMega");
		String boardType = arduino.getBoardType();

		arduino.setBoardMega();

		assertEquals(Arduino.BOARD_TYPE_MEGA, arduino.getBoardType());

		List<PinDefinition> pins = arduino.getPinList();
		assertEquals(70, pins.size());

		arduino.setBoard(boardType);
	}

	@Test
	public void testSetBoardUno() {
		log.info("testSetBoardUno");
		String boardType = arduino.getBoardType();

		arduino.setBoardUno();

		assertEquals(Arduino.BOARD_TYPE_UNO, arduino.getBoardType());

		List<PinDefinition> pins = arduino.getPinList();
		assertEquals(20, pins.size());

		arduino.setBoard(boardType);
	}

	public static class JUnitListener extends RunListener {

		public void testAssumptionFailure(Failure failure) {
			log.info("testAssumptionFailure");
		}

		public void testFailure(Failure failure) {
			log.info("testFailure");
		}

		public void testFinished(Description description) {
			log.info("testFinished");
		}

		public void testIgnored(Description description) {
			log.info("testIgnored");
		}

		public void testRunFinished(Result result) {
			log.info("testRunFinished");
		}

		public void testRunStarted(Description description) {
			log.info("testRunStarted");
		}

		public void testStarted(Description description) {
			log.info("testStarted");
		}
	}

	@Override
	public boolean isLocal() {
		return true;
	}

	@Override
	public String getName() {
		return "arduinoTest";
	}

	@Override
	public void onPinArray(PinData[] pindata) {
		log.info("onPinArray size {}", pindata.length);
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

			LoggingFactory.init(Level.INFO);
			// Runtime.start("webgui", "WebGui");
			// Runtime.start("gui", "GUIService");

			// test a "real" arduino
			userVirtualHardware = false;
			port = "COM10";
			// port = "COM4";
			// port = "COM99";

			ArduinoTest test = new ArduinoTest();
			ArduinoTest.setUpBeforeClass();

			arduino.record();

			if (virtual != null) {
				virtual.connect(port);
			}
			arduino.connect(port);

			test.testServoAttachServoInteger();

			// arduino.setBoardUno(); always better to "not" set

			// Runtime.start("webgui", "WebGui");
			test.enablePin = (arduino.getBoardType().contains("mega")) ? 54 : 15; // A0
																					// for
																					// Mega
			test.testEnableBoardStatus();
			test.testEnablePinInt();

			boolean b = true;
			if (b) {
				return;
			}

			// test specific method
			test.testServoAttachServoInteger();

			// run junit as java app
			JUnitCore junit = new JUnitCore();
			Result result = junit.run(ArduinoTest.class);
			log.info("Result was: {}", result);

			Runtime.dump();

		} catch (Exception e) {
			Logging.logError(e);
		}
	}

}
