package org.myrobotlab.service.interfaces;

public interface SensorController extends DeviceController {
	
	public void sensorActivate(SensorControl sensor, Object... conf);

	public void sensorDeactivate(SensorControl sensor);	
	
	// FIXME - should this be propegated up to DeviceController ??
	public void connect(String port) throws Exception;

	// FIXME - should this be propegated up to DeviceController ??
	public boolean isConnected();

}
