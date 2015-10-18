package ev3ObjectRecognition;


import lejos.ev3.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.hardware.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class ObjectDetection extends Thread{

	private SampleProvider colorSensor;
	private float[] colorData;
	private SampleProvider usSensor;
	private float[] usData;
	private static final long TIME_PERIOD = 10;
	
	
	// constructor
	public ObjectDetection(SampleProvider colorSensor, float[] colorData, SampleProvider usSensor, float[] usData) {
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.usSensor = usSensor;
		this.usData = usData;
	}
	
	// run method (required for Thread)
		public void run() {
			long correctionStart, correctionEnd;	
			double threshold = 20;
			
			while (true) {
				correctionStart = System.currentTimeMillis();
				colorSensor.fetchSample(colorData, 0);
				
				//UNKNOWN OBJECT WITHIN THRESHOLD
				if (getFilteredData() < threshold){
				LocalEV3.get().getTextLCD().clear();
				LocalEV3.get().getTextLCD().drawString("OBJECT DETECTED", 3, 5);	
				
					//Detection of Blue Styrofoam
					if (colorData[0]*100 < 35){
						Sound.beep(); 	
						LocalEV3.get().getTextLCD().drawString("BLOCK", 3, 6);
						}
					// OTHER OBJECT
					else {
						LocalEV3.get().getTextLCD().drawString("NOT BLOCK", 3, 6);
					 
						}
				 
				}
				
				correctionEnd = System.currentTimeMillis();
				if (correctionEnd - correctionStart < TIME_PERIOD) {
					try {
						Thread.sleep(TIME_PERIOD
								- (correctionEnd - correctionStart));
					} catch (InterruptedException e) {
					}
				}
			}
		}
		
		private float getFilteredData() {
			usSensor.fetchSample(usData, 0);
			float distance = usData[0]*100;
					
			return distance;
		}
		
	
}
