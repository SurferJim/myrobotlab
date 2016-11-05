package org.myrobotlab.arduino;

/**
 * Status data for the running MRLComm sketch. This data will be returned
 * from the sketch to Java-land to report on the speed and current free
 * memory of the Microcontroller
 */
public class BoardStatus {
	public Integer deviceCount; // deviceList with types
	// FIXME - list of current devices ids & their types ?
	// ie the deviceList
	// List<Integer, Integer>
	public Integer sram;
	public Long us;

	public BoardStatus(Long us, Integer sram, Integer deviceCount) {
		this.us = us;
		this.sram = sram;
		this.deviceCount = deviceCount;
	}
}
