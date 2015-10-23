package ev3ObjectRecognition;

import lejos.hardware.*;
import java.util.Arrays;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

//This class is mostly run after the "scan" class, its purpose is to make the robot
//Go to a block after it has recognize one and find what type of block it is
public class BlockRecognition extends Thread {
	public final Object lock = new Object(); // for blocking method
	private static final int TIME_PERIOD = 20;
	private static final int FORWARD_SPEED = 100;
	private static final int ROTATE_SPEED = 100;
	private static final int MIN_DISTANCE = 4; // how far from block to stop
	private static final double RIGHT_RADIUS = 2.1;
	private static final double LEFT_RADIUS = 2.1;
	private static final double WIDTH = 15.6;
	private static final double CM_ERR = 1.5; // allowed error in position
	private Odometer odo;
	private boolean isStyro = false; // initialize to false
	private boolean isCinder = false;
	private boolean isFinished;
	private SampleProvider usSensor;
	private SampleProvider colorSensor;
	private float[] colorData;
	private float[] usData;
	private int styrofoamColor[] = new int[] {5,9};
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor usMotor;
	Navigation nav;

	private int redValue, blueValue; // color readings

	private int distance, medianDistance;

	int[] distanceArray = new int[5];
	int[] sortedArray = new int[5];
	private boolean finisheMOFO = false;

	public BlockRecognition(Odometer odo, SampleProvider usSensor, float[] usData, SampleProvider colorSensor, float[] colorData,
			EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor leftMotor){
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.colorSensor = colorSensor;
		this.colorData = colorData;	
		this.rightMotor = rightMotor;
		this.leftMotor = leftMotor;
		nav = new Navigation(odo);
	}

	
	//This class makes the robot go to the detected object and see if it is a wooden block or styrofoam.
	public void startRun(){

		long timeStart, timeEnd;

		double xInit = odo.getX(); // starting x position
		double yInit = odo.getY(); // starting y position

		leftMotor.forward();
		rightMotor.forward();

		while (true){
			timeStart = System.currentTimeMillis();

			// If the block is within the stop distance
			if (getFilteredData() <= MIN_DISTANCE) {
				Stop();
				Delay.msDelay(500);
				setBlockType(); // check if block is styrofoam

				if (isStyro) {
					grabBlock(); //if it is take it to (70,70)
				} else {
					break;
				}

			} else {
				//goForward();
				isCinder = false;
				isStyro = false;
			}

			timeEnd = System.currentTimeMillis();
			if (timeEnd - timeStart < TIME_PERIOD) {
				try {
					Thread.sleep(TIME_PERIOD - (timeEnd - timeStart));
				} catch (InterruptedException e) {
				}
			}
		}
		/*
		while (isNotThereYet(xInit, yInit)) {
			goBackward();
		}
		 */

		nav.travelBackwardTo(xInit, yInit);
		//Stop();
		//
		//isFinished = true;
		return;
	}

	// getters
	public boolean getIsStyro() {
		return isStyro;
	}

	public boolean getIsCinder() {
		return isCinder;
	}

	// determines the type of the block using the Color ID provided by the sensor
	public void setBlockType() {
		Delay.msDelay(500);
		colorSensor.fetchSample(colorData, 0);
		float lightValue = colorData[0];
		LCD.drawInt((int)lightValue, 7, 7);

		if (lightValue > styrofoamColor[0] && lightValue < styrofoamColor[1] ) {
			isStyro = true;
			isCinder = false;
			Sound.beep();
		} else {
			Sound.beep();
			Sound.beep();
			isCinder = true;
			isStyro = false;
		}
	}

    //Go foward infinetely
	public void goForward() {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
	}

	// Go backward indefinitely
	public void goBackward() {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.backward();
		rightMotor.backward();
	}

	// stops the robot
	public void Stop() {
		leftMotor.stop();
		rightMotor.stop();
	}

	// Go around the block to take it and bring it to final position
	public void grabBlock() {
		goSetDistance(-10); // get the robot to move backwards so that it can
		// then either move around a wooden block or adjust
		// its position to push a styrofoam block

		turn(-90); // CCW
		goSetDistance(10);
		turn(90); // CW
		goSetDistance(10);
		
		//Since we only have one arm on the left, make the robot do a full turn to have the block in possession
		leftMotor.rotate(convertAngle(LEFT_RADIUS, WIDTH, 360), true);
		rightMotor.rotate(-convertAngle(RIGHT_RADIUS, WIDTH, 360), false);

		// Go to position (70,70)
		nav.travelTo(70, 70);
	}

	// travels a specific distance
	public void goSetDistance(double distance) {
		leftMotor.setSpeed(-FORWARD_SPEED);
		rightMotor.setSpeed(-FORWARD_SPEED);
		rightMotor.rotate(convertDistance(RIGHT_RADIUS, distance), true);
		leftMotor.rotate(convertDistance(LEFT_RADIUS, distance), false);
	}

	// turns the robot
	public void turn(double angle) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		leftMotor.rotate(convertAngle(LEFT_RADIUS, WIDTH, angle), true);
		rightMotor.rotate(-convertAngle(RIGHT_RADIUS, WIDTH, angle), false);
	}

	// helper method to convert the distance each wheel must travel
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// helper method to convert the angle each motor must rotate
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	// returns a boolean telling whether the robot is at target location
	public boolean isNotThereYet(double x, double y) {
		return Math.abs(x - odo.getX()) > CM_ERR
				|| Math.abs(y - odo.getY()) > CM_ERR;
	}

	public boolean isDoneForNow() {
		return isFinished;
	}

	//Return distance values from the UltrasonicSensor
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;

		return distance;
	}

	public boolean finishedMOFO(){
		return finisheMOFO;
	}

}
