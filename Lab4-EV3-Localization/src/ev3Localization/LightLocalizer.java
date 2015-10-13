package ev3Localization;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalizer {
	private Odometer odo;
	private SampleProvider colorSensor;
	private float[] colorData;
	public static int ROTATION_SPEED = 100;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private int lineCount = 0;
	private double angleLines[] = new double [4];
	// deviation erro
	private double k;
	//distance (cm) from the light sensor to center of rotation
	private double d = 12.65;

	Navigation navigation;
	
	public LightLocalizer(Odometer odo, SampleProvider colorSensor, float[] colorData, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.navigation = new Navigation(odo);
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	public void doLocalization() {	
		Sound.beepSequenceUp();
		
		navigation.start();
		
		
		
		//navigation.turnTo(45, true);
		//navigation.goForward(15);
		
		
		
		 double position[] = {0,0,45};
		boolean update[] = {true,true,true};
				odo.setPosition(position, update); 
		//while (!lineDetected())
		
		Sound.beepSequenceUp();
		/*
		// set new origin
		double position[] = {0,0,0};
		boolean update[] = {true,true,false};
				odo.setPosition(position, update);
				*/
				
				// start rotating counterclockwise
				leftMotor.setSpeed(ROTATION_SPEED);
		        rightMotor.setSpeed(ROTATION_SPEED);
			    leftMotor.backward();
		        rightMotor.forward();
		        
		 while (lineCount<4){
			 if(lineDetected()){
				 angleLines[lineCount]= odo.getAng();
				 lineCount++;
				 Sound.beep();
				 Delay.msDelay(500);
				 
				 //test
				 leftMotor.stop();
				 rightMotor.stop();
				 Delay.msDelay(1500);
				    leftMotor.backward();
			        rightMotor.forward();
				 
				 
			 }
		 }
		 
		 leftMotor.stop();
		 rightMotor.stop();
		        
		 Sound.beepSequenceUp();       
		 
		 //Using formulas from the Localization slides
		 //Find the x and y position and theta using trigonometry
	     double deltaY = angleAverage(angleLines[0], angleLines[2]);
	     double deltaX = angleAverage(angleLines[1], angleLines[3]);
	     double x = -(d * Math.cos(deltaY));
	     double y = -(d * Math.cos(deltaX));
	     
	    // double newTheta = (deltaY / 2 + 180 - angleLines[3])+ odo.getAng();
	     double newTheta = 90 - angleLines[3] + 180 + deltaY;
	     
	     double newPosition[]={x, y, newTheta};
	    // boolean update[] = {true, true, true};
	     odo.setPosition(newPosition, new boolean[] {true, true, true});
	     
	     Delay.msDelay(2000);
	     navigation.travelTo(0, 0);
	     navigation.turnTo(0,true);
		 
		// drive to location listed in tutorial
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
		 
		 
	}
	
	private boolean lineDetected(){
		colorSensor.fetchSample(colorData, 0);
		if ( colorData[0]*100 < 35){
			return true;
		}
		return false;
	}
	
	private double angleDifference(double a, double b){
		return Math.min((360) - Math.abs(a - b), Math.abs(a - b));
	}
	
	private double angleAverage(double a, double b){
		double x = Math.abs(a-b);
		double result=0;
				if (x < 180){
				   result = ((a + b) / 2);
				}
				else if (x != 180) {
				   result = ((a + b) / 2) + 180;
				}
				  else {
					  result = 180;
				  }
				   //two solutions are possible

				return result % 360;
	}


}
