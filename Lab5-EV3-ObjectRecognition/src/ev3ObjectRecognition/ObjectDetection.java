package ev3ObjectRecognition;


import lejos.ev3.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.hardware.*;
import lejos.robotics.SampleProvider;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

//This method is used for PartA of Lab5, it doesn't involve the motors, only the sensors and the screen
//To display a description of objects displayed in front of the robot
public class ObjectDetection extends Thread{

	private SampleProvider colorSensor;
	private float[] colorData;
	private SampleProvider usSensor;
	private float[] usData;
	private static final long TIME_PERIOD = 10;
	private double threshold = 4;
	private double upper_threshold = 20;
	private int styrofoamColor[] = new int[] {5,9};


	public ObjectDetection(SampleProvider colorSensor, float[] colorData, SampleProvider usSensor, float[] usData) {
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.usSensor = usSensor;
		this.usData = usData;
	}

	// This method differentiate a Wooden block from a Styrofoam block, used in PartA of Lab5
	public void run() {
		long correctionStart, correctionEnd;	
		int buttonChoice = 0;
		while (buttonChoice == 0) {
			correctionStart = System.currentTimeMillis();

			LCD.clear();
			
			//If there is an object in range, bring it closer so the colorSensor can see what it is
			if (getFilteredData() < upper_threshold){
				if (getFilteredData() > threshold) {
					LocalEV3.get().getTextLCD().drawString("Bring Closer", 1, 1);
				}
				LocalEV3.get().getTextLCD().clear();
				LocalEV3.get().getTextLCD().drawString("OBJECT DETECTED", 3, 5);	

				//Detection of Blue Styrofoam
				if ( getColorID() < styrofoamColor[1] && getColorID()>styrofoamColor[0] ){
					Sound.beep(); 	
					LocalEV3.get().getTextLCD().drawString("Styrofoam Block", 4, 6);
				}
				// If it is not a styrofoam, it is a wooden block
				else {
					LocalEV3.get().getTextLCD().drawString("Wooden block", 3, 6);

				}

			}

			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < TIME_PERIOD) {
				try {
					Thread.sleep(TIME_PERIOD
							- (correctionEnd - correctionStart));
				} 
				catch (InterruptedException e) {
				}
			}
			buttonChoice = Button.waitForAnyPress(10);
		}
		LCD.clear();
	}
	
	//Return distance values from the UltrasonicSensor
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;
		LCD.drawString("Dist:", 0, 3);
		LCD.drawInt((int) distance, 3, 3);
		return distance;
	}

	//Return the ID of the color of the object in front 
	private float getColorID(){
		colorSensor.fetchSample(colorData, 0);
		float color = colorData[0];
		LCD.drawString("Col:", 0, 3);
		LCD.drawInt((int) color, 3, 3);
		return color;
	}


}
