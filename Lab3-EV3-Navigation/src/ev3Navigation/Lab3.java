/* Lab 3 Navigation
 * Group 7
 * Sebastien Arrese & Arnold Kokoroko
 * This is the Main class. This makes the robot run.
 */

package ev3Navigation;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Lab3 {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor connected to input port S1
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final Port usPort = LocalEV3.get().getPort("S1");

	// Constants
	public static final double WHEEL_RADIUS = 2.17;
	public static final double TRACK = 15.175;



	public static void main(String[]args){
		int buttonChoice;
		// Instantiating Odometer and Odometer display

		final TextLCD t = LocalEV3.get().getTextLCD();
		Odometer odometer = new Odometer(leftMotor, rightMotor);
		Navigation navigation = new Navigation(odometer, leftMotor, rightMotor);
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer,t);
		ObstacleAvoidanceNavigation navigationObstacle = new ObstacleAvoidanceNavigation(odometer, leftMotor, rightMotor);

		// Setup ultrasonic sensor
		// Note that the EV3 version of leJOS handles sensors a bit differently.
		// There are 4 steps involved:
		// 1. Create a port object attached to a physical port (done already above)
		// 2. Create a sensor instance and attach to port
		// 3. Create a sample provider instance for the above and initialize operating mode
		// 4. Create a buffer for the sensor data

		//@SuppressWarnings("resource")							    // Because we don't bother to close this resource
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);		// usSensor is the instance
		SampleProvider usDistance = usSensor.getMode("Distance");	// usDistance provides samples from this instance
		float[] usData = new float[usDistance.sampleSize()];		// usData is the buffer in which data are returned

		// Setup Ultrasonic Poller									// This thread samples the US and invokes
		UltrasonicPoller usPoller = null;							// the selected controller on each cycle
		UltrasonicPoller usPoller2 = null;

		do {
			// Clear the display
			t.clear();

			// Ask the user what mode to choose
			t.drawString("Left, Navigate ", 0, 0);	
			t.drawString("Right, Obstacle", 0, 1);
			buttonChoice = Button.waitForAnyPress();

		} 


		while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			odometer.start();
			odometryDisplay.start();
			navigation.start();
		}

		else if (buttonChoice == Button.ID_RIGHT) {
			odometer.start();
			odometryDisplay.start();
			usPoller = new UltrasonicPoller(usDistance, usData, navigationObstacle);

			usPoller.start();
			navigationObstacle.start();
		}

		// to end the program
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);


	}

}
