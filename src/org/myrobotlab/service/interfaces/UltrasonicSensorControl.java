package org.myrobotlab.service.interfaces;

public interface UltrasonicSensorControl extends DeviceControl {
	
	public void startRanging();
	
	public void startRanging(int timeoutMS);
	
	public void stopRanging();
	
	public Integer onUltrasonicSensorData(Integer us);

}
