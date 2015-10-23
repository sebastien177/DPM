package ev3ObjectRecognition;


import lejos.*;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

//This code is moslty taken from the USLocalizer done in Lab5
//It makes the robot find it's initial position and orientation
public class USLocalizer {
	public enum LocalizationType { // The two localization type available
		FALLING_EDGE, RISING_EDGE
	};

	//Variable declaration
	public static double ROTATION_SPEED = 30;
	public static double SENSOR_DISTANCE = 8.0; // Distance of the sensor from the robot's center
	private static double turnError = 1.5; 
	private double leftRadius, rightRadius, width;
	private double forwardSpeed, rotationSpeed;
	private double threshold = 41; // The threshold value when a robot sees the wall
	private Odometer odo;
	private LocalizationType locType;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private SampleProvider usSensor;
	private float[] usData;


	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData,
			LocalizationType locType, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, double wheelRadius, double width) {
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usSensor = usSensor;
		this.usData = usData;
		this.locType = locType;
		this.leftRadius= wheelRadius;
		this.rightRadius = wheelRadius;
		this.width = width;
	}

	// This class makes the robot localize at the start of the lab
	//It makes the robot orient itself to an angle of 45 degrees at the position (0,0)
	public void doLocalization() {

		Navigation nav = new Navigation(this.odo);
		// These will be the angles of the robot from both walls
		double angleA, angleB;

		//Initialize the robot position to 0
		odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] { true,
				true, true });
		// The localization type used for the lab will mostly by Falling Edge
		if (locType == LocalizationType.FALLING_EDGE) {

			// Rotate the robot until it sees no wall
			while (getFilteredData() <= threshold) {
				setRotationSpeed(-25);

			}
			// Determining the angle when it sees no wall
			angleA = odo.getAng();
			Delay.msDelay(500);

			// Keep rotating until the robot sees a wall, then latch the angle
			while (getFilteredData() + 10 >= threshold) {

				setRotationSpeed(-35);

			}

			// Switch direction and wait until it sees no wall
			while (getFilteredData() <= threshold) {

				setRotationSpeed(10);

			}
			// Tracking the angle when it senses the second wall
			angleB = odo.getAng();

			// Stop the robot after it senses two walls
			setRotationSpeed(0);

			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'

			double theta = 0;
			if (angleA > angleB) {

				theta = 45 - (angleA + angleB) / 2;
				theta += odo.getAng();
			} else {

				theta = 230 - (angleA + angleB) / 2;
			}

			// Updating the odometer's angle after it does the ultrasonic localization
			odo.setPosition(new double[] { 0.0, 0.0, theta }, new boolean[] {
					true, true, true });

			for (int i = 0; i < 50; i++) {
				setRotationSpeed(-10);
			}

			// This rotation is to face the wall directly to get its x position
			while (odo.getAng() > 270) {
				setRotationSpeed(-20);


			}

			setRotationSpeed(0);

			// Calculating the x position taking into consideration the sensor distance
			double xPosition = 2*(getFilteredData() + SENSOR_DISTANCE);

			// Updating the x position of the robot after facing the wall
			odo.setPosition(new double[] { -xPosition, 0.0, odo.getAng() },
					new boolean[] { true, false, false });

			// This keeps turning to face another wall to get its y position
			while (odo.getAng() > 180) {

				setRotationSpeed(-20);
			}

			setRotationSpeed(0);

			// Calculating the y position taking into consideration the sensor distance
			double yPosition = 2*((getFilteredData() + SENSOR_DISTANCE));


			// Setting the final position of the robot after localizing
			odo.setPosition(
					new double[] { 0.0, -yPosition, odo.getAng() },
					new boolean[] { false, true, false });

			// Turning to the 0 degrees so it is facing the same orientation as the origin
			while (180.0 - odo.getAng() + turnError < 180) {

				setRotationSpeed(-30);

			}
			// Stop the robot after reaching the origin
			setRotationSpeed(0);
			//nav.travelTo(0, 0);
			odo.setPosition(
					new double[] { 0.0, 0.0, odo.getAng()},
					new boolean[] { true, true, true });


		}
	}

	// This is a getting to get the filtered ultrasonic sensor (used for display purposes)
	public int getData() {
		return getFilteredData();
	}

	// This is the filter for the ultrasonic sensor. It returns a filtered sensor reading
	private int getFilteredData() {

		// wait for the ping to complete
		try {
			Thread.sleep(25);
		} 
		catch (InterruptedException e) {

		}
		usSensor.fetchSample(usData, 0);
		int distance = (int) (usData[0]*100);

		LCD.drawInt((int) distance, 3, 3);
		LCD.drawInt((int) SENSOR_DISTANCE, 6, 6);

		// This is the filter. If the distance is more than 35, it is considered to be "infinite" so set the distance
		// to be 35

		if (distance > 42) {

			distance = 42;

		}

		return distance;

	}

	public void setForwardSpeed(double speed) {
		forwardSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}

	public void setRotationSpeed(double speed) {
		rotationSpeed = speed;
		setSpeeds(forwardSpeed, rotationSpeed);
	}

	public void setSpeeds(double forwardSpeed, double rotationalSpeed) {
		double leftSpeed, rightSpeed;

		this.forwardSpeed = forwardSpeed;
		this.rotationSpeed = rotationalSpeed; 

		leftSpeed = (forwardSpeed + rotationalSpeed * width * Math.PI / 360.0) *
				180.0 / (leftRadius * Math.PI);
		rightSpeed = (forwardSpeed - rotationalSpeed * width * Math.PI / 360.0) *
				180.0 / (rightRadius * Math.PI);

		// set motor directions
		if (leftSpeed > 0.0)
			this.leftMotor.forward();
		else {
			this.leftMotor.backward();
			leftSpeed = -leftSpeed;
		}

		if (rightSpeed > 0.0)
			this.rightMotor.forward();
		else {
			this.rightMotor.backward();
			rightSpeed = -rightSpeed;
		}

		// set motor speeds
		this.leftMotor.setSpeed((int)leftSpeed);

		this.rightMotor.setSpeed((int)rightSpeed);
	}


}