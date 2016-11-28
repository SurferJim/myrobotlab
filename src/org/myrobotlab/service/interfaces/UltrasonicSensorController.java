package org.myrobotlab.service.interfaces;

public interface UltrasonicSensorController extends DeviceController {
	
	// > ultrasonicSensorAttach/deviceId/triggerPin/echoPin
	public void ultrasonicSensorAttach(UltrasonicSensorControl sensor, Integer triggerPin, Integer echoPin);
	
	// > ultrasonicSensorStartRanging/deviceId/b32 timeout
	public void ultrasonicSensorStartRanging(UltrasonicSensorControl sensor, Integer timeout);
	
	// > ultrasonicSensorStopRanging/deviceId
	public void ultrasonicSensorStopRanging(UltrasonicSensorControl sensor);

	public boolean isConnected();

	public void connect(String port);

}
