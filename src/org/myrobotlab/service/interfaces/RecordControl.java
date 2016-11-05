package org.myrobotlab.service.interfaces;

public interface RecordControl {
	
	public void record() throws Exception;
	
	public void stopRecording();
	
	public boolean isRecording();
	
	public void setFormat(String format) throws Exception;

}
