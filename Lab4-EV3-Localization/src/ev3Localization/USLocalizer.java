package ev3Localization;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class USLocalizer {
	public enum LocalizationType { FALLING_EDGE, RISING_EDGE };
	public static double ROTATION_SPEED = 100;
	public static final double DEFAULT_LEFT_RADIUS = 2.145;
	public static final double DEFAULT_RIGHT_RADIUS = 2.145;
	public static final double DEFAULT_WIDTH = 15.25;
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;
	private LocalizationType locType;
	private EV3LargeRegulatedMotor leftMotor,rightMotor;
	
	public USLocalizer(Odometer odo,  SampleProvider usSensor, float[] usData, LocalizationType locType,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		
	}
	
	public void doLocalization() {
		double [] pos = new double [3];
		double angleA, angleB;
		double threshold = 20;
		odo.setPosition(new double[] { 0.0, 0.0, 0},
				new boolean[] { true, true, true });
		
		if (locType == LocalizationType.FALLING_EDGE) {
			// rotate the robot until it sees no wall
			while (getFilteredData() <= threshold) {
				rotateClockwise();
			}	
			Sound.beep();
			
			// keep rotating until the robot sees a wall, then latch the angle

			while (getFilteredData()>= threshold){
				rotateClockwise();

			}
			Sound.buzz();
			angleA = odo.getAng();
			
			leftMotor.stop();
	        rightMotor.stop();
	        
			// switch direction and wait until it sees no wall
			while (getFilteredData() <= threshold) {
				rotateCounterClock();
			}
			Sound.beep();
			//To be sure to not to record the same wall
			Delay.msDelay(500);
			
			// keep rotating until the robot sees a wall, then latch the angle
			while (getFilteredData() >= threshold){
				rotateCounterClock();
			}
			Sound.buzz();
			angleB = odo.getAng();
			
			leftMotor.stop();
	        rightMotor.stop();

			
			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			double deltaTheta = 0;
			if (angleA > angleB) {
				deltaTheta = 225 - (angleAverage(angleA, angleB));
			} 
			else {
				deltaTheta = 45 - (angleAverage(angleA, angleB));
			}
			// update the odometer position (example to follow:)
			odo.setPosition(new double[] { 0.0, 0.0, odo.getAng() + deltaTheta },
					new boolean[] { true, true, true });
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall.
			 * This is very similar to the FALLING_EDGE routine, but the robot
			 * will face toward the wall for most of it.
			 */
			
			// rotate the robot until it sees a wall
						while (getFilteredData() >= threshold) {
							rotateClockwise();
						}	
						
						// keep rotating until the robot sees no wall, then latch the angle
						while (getFilteredData() <= threshold){
							rotateClockwise();
						}
						angleA = odo.getAng();
						
						leftMotor.stop();
				        rightMotor.stop();
						
						// switch direction and wait until it sees a wall
						while (getFilteredData() >= threshold) {

							rotateCounterClock();

						}
						
						// keep rotating until the robot sees no wall, then latch the angle
						while (getFilteredData() <= threshold){
							rotateCounterClock();
						}

						
						angleB = odo.getAng();
						
						//Stop Rotating
						leftMotor.stop();
				        rightMotor.stop();
						
						// angleA is clockwise from angleB, so assume the average of the
						// angles to the right of angleB is 45 degrees past 'north'
						double deltaTheta = 0;
						if (angleA > angleB) {
							deltaTheta = 225 - (angleAverage(angleA, angleB));
						} 
						else {
							deltaTheta = 45 - (angleAverage(angleA, angleB));
						}
						// update the odometer position (example to follow:)
						odo.setPosition(new double[] { 0.0, 0.0, odo.getAng() + deltaTheta },
								new boolean[] { true, true, true });
					}
	}
	
	public void rotateClockwise(){
		rightMotor.setSpeed((int) ROTATION_SPEED); 
		leftMotor.setSpeed((int) ROTATION_SPEED);
		leftMotor.forward();
        rightMotor.backward();
	}
	public void rotateCounterClock(){
		rightMotor.setSpeed((int) ROTATION_SPEED); 
		leftMotor.setSpeed((int) ROTATION_SPEED);
		leftMotor.backward();
        rightMotor.forward();
	}
	
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;
				
		return distance;
	}
	
	//Method takes two angle a and b, and find the smaller average angle between the two possible ones
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

				return result % 360;
	}

}