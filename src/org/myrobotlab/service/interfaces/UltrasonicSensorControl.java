package org.myrobotlab.service.interfaces;

public interface UltrasonicSensorControl extends SensorDataListener, DeviceControl {
	
	public void attach(SensorController controller, Object...conf);
	
	public void startRanging();
	
	public void startRanging(int timeoutMS);
	
	public void stopRanging();

}
