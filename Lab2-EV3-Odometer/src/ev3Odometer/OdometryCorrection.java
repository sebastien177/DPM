package ev3Odometer;

//OdometryCorrection.java

import lejos.ev3.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.hardware.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;



public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private static final int THRESHOLD = 520;
	private static final double OFFSET = 5.5; // difference between sensor and wheels
	private static final double HALF_SQUARE = 15.24;
	private int sensorCount = 0;
	private Odometer odometer;
	private EV3ColorSensor lightSensor = new EV3ColorSensor(SensorPort.S1);
	
	


	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
		lightSensor.setFloodlight(true);
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;	

		while (true) {
			correctionStart = System.currentTimeMillis();

			// put your correction code here
			
			//detection of black lines
			 if (lightSensor.getFloodlight() <= THRESHOLD){
				//keep track of when line is crossed
				 sensorCount++;
				
			
			}

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}

}