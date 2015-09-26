package ev3Odometer;

//OdometryCorrection.java

import lejos.ev3.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.hardware.*;
import lejos.robotics.SampleProvider;



public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	
	private static final Port csPort = LocalEV3.get().getPort("S1");

	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		SensorModes colorSensor =  new EV3ColorSensor(csPort);
		colorSensor.setCurrentMode("Color ID");
		

		while (true) {
			correctionStart = System.currentTimeMillis();

			// put your correction code here

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