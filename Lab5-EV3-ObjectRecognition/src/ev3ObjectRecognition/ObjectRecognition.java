package ev3ObjectRecognition;
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;


public class ObjectRecognition {
	private static EV3ColorSensor lightSensor;
	private static EV3UltrasonicSensor usSensor;
	float[] sampleRed = {0};
	
	public ObjectRecognition(EV3ColorSensor lightSensor, EV3UltrasonicSensor usSensor){
		this.lightSensor = lightSensor;
		this.usSensor = usSensor;		
	}
	
	public boolean isBlockBlue(){
		lightSensor.getRedMode().fetchSample(sampleRed, 0);
		boolean isDetecting = true;
		while(isDetecting){
			 // If value read is over * *, then it is a blue block
			if(sampleRed[0]*100 >= 250){
				Sound.beep();
				LocalEV3.get().getTextLCD().drawString("Blue styrofoam block", 3, 5);				
				return true;
				
			}
			else{ // If it isn't a blue a bloack
				LocalEV3.get().getTextLCD().drawString("Wooden block", 3, 5);	
				
			}
			
			isDetecting = false;			
		}
		return false;
		
	}
	
}
