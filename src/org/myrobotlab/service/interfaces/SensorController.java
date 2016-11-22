package org.myrobotlab.service.interfaces;

public interface SensorController extends DeviceController {
	
	public void sensorAttach(UltrasonicSensorControl sensor, int trigPin, int echoPin);
	
	public void sensorActivate(UltrasonicSensorControl sensor, Object... conf);

	public void sensorDeactivate(UltrasonicSensorControl sensor);	
	
	// FIXME - should this be propegated up to DeviceController ??
	public void connect(String port) throws Exception;

	// FIXME - should this be propegated up to DeviceController ??
	public boolean isConnected();

}
