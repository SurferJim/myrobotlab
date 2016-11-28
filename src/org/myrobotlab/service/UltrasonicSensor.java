package org.myrobotlab.service;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

import org.myrobotlab.framework.Service;
import org.myrobotlab.framework.ServiceType;
import org.myrobotlab.logging.Level;
import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.logging.Logging;
import org.myrobotlab.logging.LoggingFactory;
import org.myrobotlab.service.interfaces.DeviceController;
import org.myrobotlab.service.interfaces.RangeListener;
import org.myrobotlab.service.interfaces.UltrasonicSensorControl;
import org.myrobotlab.service.interfaces.UltrasonicSensorController;
import org.slf4j.Logger;

/**
 * 
 * UltrasonicSensor - This will read data from an ultrasonic sensor module
 * connected to an android.
 * 
 * A device which uses the UltrasonicSensor would implement RangeListener.
 * UltrasonicSensor implements RangeListener just for testing purposes
 *
 */
public class UltrasonicSensor extends Service implements RangeListener, UltrasonicSensorControl {

	private static final long serialVersionUID = 1L;

	public final static Logger log = LoggerFactory.getLogger(UltrasonicSensor.class);

	public final Set<String> types = new HashSet<String>(Arrays.asList("SR04", "SR05"));
	private int pings;
	
	private Integer trigPin = null;
	private Integer echoPin = null;
	private String type = "SR04";

	private Integer lastRaw;
	private Integer lastRange;

	// for blocking asynchronous data
	private boolean isBlocking = false;

	transient private BlockingQueue<Integer> data = new LinkedBlockingQueue<Integer>();

	private transient UltrasonicSensorController controller;

	String controllerName;

	public UltrasonicSensor(String n) {
		super(n);
	}

	// ---- part of interfaces begin -----

	// Uber good - .. although this is "chained" versus star routing
	// Star routing would be routing from the Arduino directly to the Listener
	// The "chained" version takes 2 thread contexts :( .. but it has the
	// benefit
	// of the "publishRange" method being affected by the Sensor service e.g.
	// change units, sample rate, etc
	// FIXME - NOT SERVICE .. possibly name or interface but not service
	public void addRangeListener(Service service) {
		addListener("publishRange", service.getName(), "onRange");
	}
	
	public void attach(String port, int trigPin, int echoPin) throws Exception {
		if (controller == null){
			// no controller specified - create a default (Arduino) controller 
			controller = (UltrasonicSensorController)startPeer("controller");			
		}
		
		// FIXME -  make sure we're connected
		controller.connect(port);

		this.trigPin = trigPin;
		this.echoPin = echoPin;
		controller.ultrasonicSensorAttach(this, trigPin, echoPin);
	}

	public void attach(UltrasonicSensorController controller, int trigPin, int echoPin) throws Exception {
		if (!controller.isConnected()){
			error("cannot attach if controller is not connected");
		}
		this.controller = controller;
		this.trigPin = trigPin;
		this.echoPin = echoPin;
		// FIXME - controller.ultrasonicSensorAttach(this, trigPin, echoPin);
		// controller.deviceAttach(this, trigPin, echoPin);
	}

	// FIXME - should be MicroController Interface ..
	public UltrasonicSensorController getController() {
		return controller;
	}

	public int getEchoPin() {
		return echoPin;
	}

	public int getTriggerPin() {
		return trigPin;
	}

	@Override
	public void onRange(Long range) {
		log.info(String.format("RANGE: %d", range));
	}

	/* FIXME !!! IMPORTANT PUT IN INTERFACE & REMOVE SELF FROM ARDUINO !!! */
	public Integer publishRange(Integer duration) {

		++pings;

		lastRange = duration / 58;

		log.info("publishRange {}", lastRange);
		return lastRange;
	}

	public long range() {
		return range(10);
	}

	public long range(int timeout) {

		Integer ret = null;

		try {
			data.clear();
			startRanging(timeout);
			// sendMsg(GET_VERSION);
			ret = data.poll(timeout, TimeUnit.MILLISECONDS);
		} catch (Exception e) {
			Logging.logError(e);
		}
		data.clear(); // double tap
		return ret;// controller.pulseIn(trigPin, echoPin, timeout);
	}

	public boolean setType(String type) {
		if (types.contains(type)) {
			this.type = type;
			return true;
		}
		return false;
	}

	// ---- part of interfaces end -----

	@Override
	public void startRanging() {
		startRanging(10); // 10000 uS = 10 ms
	}

	@Override
	public void startRanging(int timeoutMS) {
		controller.ultrasonicSensorStartRanging(this, timeoutMS);
	}

	@Override
	public void stopRanging() {
		controller.ultrasonicSensorStopRanging(this);
	}

	// probably should do this in a util class
	public static int byteArrayToInt(int[] b) {
		return b[3] & 0xFF | (b[2] & 0xFF) << 8 | (b[1] & 0xFF) << 16 | (b[0] & 0xFF) << 24;
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

		ServiceType meta = new ServiceType(UltrasonicSensor.class.getCanonicalName());
		meta.addDescription("Ranging sensor");
		meta.addCategory("sensor");
		meta.addPeer("controller", "Arduino", "default sensor controller will be an Arduino");
		return meta;
	}

	public int getPings() {
		return pings;
	}

	public String getType() {
		return type;
	}
	


	// FIXME should be done in "default" interface or abstract class :P
	@Override
	public boolean isAttached() {
		return controller != null;
	}

	@Override
	public void setController(DeviceController controller) {
		this.controller = (UltrasonicSensorController)controller;
		broadcastState();
	}
	
	@Override
	public void unsetController(){
		this.controller = null;
		broadcastState();
	}
	
	public Integer onUltrasonicSensorData(Integer rawData) {
		// data comes in 'raw' and leaves as Range
		// TODO implement changes based on type of sensor SRF04 vs SRF05
		// TODO implement units preferred 
		// direct callback vs pub/sub (this needs to be handled by the framework)
		
		// FIXME - convert to appropriate range
		// inches/meters/other kubits?
		
		++pings;
		lastRaw = rawData;
		Integer range = rawData;
		if (isBlocking) {
			try {
				data.put(lastRaw);
			} catch (InterruptedException e) {
				Logging.logError(e);
			}
		}

		invoke("publishRange", range);
		return range;
	}
	
	@Override
	public void setUnits(String units) {
		// TODO Auto-generated method stub
		
	}
	
	public static void main(String[] args) {
		LoggingFactory.getInstance().configure();
		LoggingFactory.getInstance().setLevel(Level.INFO);

		try {

			UltrasonicSensor srf04 = (UltrasonicSensor)Runtime.start("srf04", "UltrasonicSensor");
			// Runtime.start("python", "Python");
			// Runtime.start("gui", "GUIService");
			
			int trigPin = 8;
			int echoPin = 7;
			
			
			
			// TODO test with externally supplied arduino
			
			srf04.attach("COM10", trigPin, echoPin);
			
			Arduino arduino = (Arduino)srf04.getController();
			arduino.enableBoardStatus(true);
			arduino.enableBoardStatus(false);
			arduino.setDebug(true);
			
			srf04.startRanging();
			
			srf04.stopRanging();
			
			arduino.setDebug(false);

		} catch (Exception e) {
			Logging.logError(e);
		}
	}


}
