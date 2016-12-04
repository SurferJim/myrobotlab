package org.myrobotlab.service.interfaces;

public interface UltrasonicSensorController extends DeviceController {
	
	public void attach(UltrasonicSensorControl control, Integer pin);
	
	// > ultrasonicSensorAttach/deviceId/triggerPin/echoPin
	public void ultrasonicSensorAttach(UltrasonicSensorControl sensor, Integer triggerPin, Integer echoPin);
	
	// > ultrasonicSensorStartRanging/deviceId/b32 timeout
	public void ultrasonicSensorStartRanging(UltrasonicSensorControl sensor, Integer timeout);
	
	// > ultrasonicSensorStopRanging/deviceId
	public void ultrasonicSensorStopRanging(UltrasonicSensorControl sensor);

	// FIXME - is the controller or MicroController ?
	public boolean isConnected();

	public void connect(String port);

}
