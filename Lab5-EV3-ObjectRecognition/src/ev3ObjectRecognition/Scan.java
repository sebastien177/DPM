package ev3ObjectRecognition;
//12h33 pour le save original
import lejos.hardware.*;
import java.util.Arrays;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Scan extends Thread{

	private SampleProvider usSensor;
	private SampleProvider colorSensor;
	private float[] colorData;
	private Odometer odo;
	private Navigation nav;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor usMotor;
	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 150;
	private static final double RIGHT_RADIUS = 2.1;
	private static final double LEFT_RADIUS = 2.1;
	private static final double WIDTH = 15.6;
	private static final double CM_ERR = 1.5;
	int[] distanceArray = new int[5]; // distance readings
	int[] sortedArray = new int[5]; // distance readings in ascending order
	boolean isFinished;
	private float[] usData;
	private static final int TIME_PERIOD = 20;
	private boolean isObject = false;
	private boolean coordinateNotReached = true;
	private boolean isDone = false;
	int FAST = 150, SLOW = 100, ACCELERATION = 4000;
	private boolean interruption = true;
	final static double DEG_ERR = 3.0;

	public Scan (SampleProvider usSensor, float[] usData, SampleProvider colorSensor, float[] colorData, Odometer odo,  EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor usMotor){
		this.usSensor = usSensor;
		this.usData = usData;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usMotor = usMotor;
	}

	public Scan (EV3LargeRegulatedMotor usMotor){
		this.usMotor = usMotor;
	}

	@SuppressWarnings("deprecation")
	public void startRun(){
		isFinished = false;		
		final Navigation nav = new Navigation(this.odo);
		final Navigation nav2 = new Navigation(this.odo);

		odo.setPosition(new double[]{0.0,0.0,45.0}, new boolean[]{true,true,true});
		turn(-45);
		//			For test	

		/*
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();
		 */
		//nav.travelTo(0, 60);

		// nav.travelTo(0, 30);

		/* 
		 Thread caca = (new Thread() {
				public void run() {
					nav.travelTo(0, 30);
				}
			});
		 caca.start();

		 Sound.beep();
		 Delay.msDelay(1000);
		 caca.interrupt();
		 Sound.beep();
		 caca.resume();
		leftMotor.stop();
		rightMotor.stop();
		Sound.beep();
		//Delay.msDelay(10000);
		 */

		usMotor.setSpeed(70); // rotate sensor fast to be useful while moving
		turnSensor(-90); // rotate sensor counterclockwise

		// assuming we're near our starting position, travel to 0,60

		/* 
		 if (Math.abs(odo.getX() - 0) < CM_ERR
						&& Math.abs(odo.getY() - 60) > CM_ERR) {
					travelTo(0, 60);

					if (isNotThereYet(60, 0)) {
						isFinished = true;
						return;
					}

					turn(90); // turn CCW
				}

				else if (Math.abs(odo.getX() - 180) > CM_ERR
						&& Math.abs(odo.getY() - 60) < CM_ERR) {
					travelTo(180, 60);

					if (isNotThereYet(60, 180)) {
						isFinished = true;
						return;
					}

					turn(-90); // turn CCW
				}

				else {
					isFinished = true;
				}
		 */

		//To know if we reached the first coordinate yet
		while (coordinateNotReached){
	
			while (!isDone){
				travelTo(0, 60);

				//If we reached the wanted coordinate, go to the next loop, we don't have the object yet
				if (isDone){
					coordinateNotReached = false;
				}

				//If there is an object inside 30cm get out of the while loop and set isObject to true, to go and get it
				if (getFilteredData()< 30) {
					FAST=30;
					while (getFilteredData()<40){
						travelTo(0,60);
					}
					setSpeeds(0, 0);
					FAST=150;
					isDone=true;
					
					
					//nav.goBackward();
					//		try{
					//caca.interrupt();
					//caca.wait();
					//caca.stop();
					//		}
					/*		catch (ThreadDeath E){
					//cuz we dont care
					//System.out.println(E);
				}
				catch (Error E){

				}
				catch (Exception A){

				}*/
					isObject = true;
				}
			}
			isDone=false;

			/*If it was an object, turn the robot and the sensor toward the object and
			 * set isFinished to start the BlockRecognition class from the main 
			 */
			if (isObject) {
				//lancer une methode

				/*leftMotor.stop();
			rightMotor.stop();*/
				usMotor.setSpeed(25);
				turnSensor(90);
				turn(90);
				Lab5.br.startRun();
				//isFinished = true;
			}
			//isFinished = false;
			//Si les deux classes run en meme temps, use synchronized(lock)
			Sound.beepSequenceUp();
			if(isObject){
				turn(-90); // turn CCW
				turnSensor(-90);
				leftMotor.forward();
				rightMotor.forward();
			}
			isObject = false;
		}

		//Object was not found, go to next coordinate
		coordinateNotReached = true;
		while (coordinateNotReached){

			travelTo(60, 60);
			

			while (!nav2.isDone()){

				//If we reached the wanted coordinate, go to the next loop, we don't have the object yet
				if (nav2.isDone()){
					coordinateNotReached = false;
				}

				//If there is an object inside 30cm get out of the while loop and set isObject to true, to go and get it
				if (getFilteredData()< 30) {
					nav2.setIsDone();
					nav2.interruptIt();
					
					//		try{
				//caca.interrupt();
					//caca.wait();
					//caca.stop();
					//		}
					/*		catch (ThreadDeath E){
						//cuz we dont care
						//System.out.println(E);
					}
					catch (Error E){

					}
					catch (Exception A){

					}*/
					isObject = true;
				}
			}
			nav2.setNotDone();

			/*If it was an object, turn the robot and the sensor toward the object and
			 * set isFinished to start the BlockRecognition class from the main 
			 */
			if (isObject) {
				//lancer une methode

				/*leftMotor.stop();
				rightMotor.stop();*/
				usMotor.setSpeed(25);
				turnSensor(90);
				turn(90);
				//This will start a new Thread in main
				isFinished = true;
			}
			isFinished = false;
			//Si les deux classes run en meme temps, use synchronized(lock)

			if(isObject){
				turn(-90); // turn CCW
				turnSensor(-90);
				nav2.goForward(5);
			}
			Sound.beepSequenceUp();

		}


		if (Math.abs(odo.getX() - 180) > CM_ERR
				&& Math.abs(odo.getY() - 60) < CM_ERR) {
			travelTo2(180, 60);

			if (isNotThereYet(60, 180)) {
				isFinished = true;
				return;
			}

			turn(90); // turn CCW
		}

		else {
			isFinished = true;
		}
	}

	public void travelTo2(double x, double y) {
		boolean isObject = false;

		long timeStart, timeEnd;

		Arrays.fill(distanceArray, 255); // initializes the distanceArray

		while (isNotThereYet(x, y)) {
			timeStart = System.currentTimeMillis();

			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.forward();
			rightMotor.forward();

			// If distance is less 
			// than 30, set isObject true
			if (getFilteredData()< 30) {
				isObject = true;
				break;
			}

			timeEnd = System.currentTimeMillis();
			if (timeEnd - timeStart < TIME_PERIOD) {
				try {
					Thread.sleep(TIME_PERIOD - (timeEnd - timeStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the detector will be
					// interrupted by another thread
				}
			}

		}

		//once an object is found
		if (isObject) {
			usMotor.setSpeed(25);
			turnSensor(90);
			turn(-90);
		}

	}



	// determines if robot is near target
	public boolean isNotThereYet(double x, double y) {
		return Math.abs(x - odo.getX()) > CM_ERR
				|| Math.abs(y - odo.getY()) > CM_ERR;
	}

	// turns the robot by a specified angle
	public void turnSensor(int degrees) {
		usMotor.rotate(degrees);
	}

	// turns the robot by a specified angle
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




	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0]*100;

		return distance;
	}

	public boolean getIsDone() {
		return isFinished;
	}

	public void usMotorSpeed(int speed){
		usMotor.setSpeed(speed);
	}
	public boolean coordinateReached(){
		return coordinateNotReached ;
	}
	
	public void travelTo(double x, double y) {
		isDone  = false;
		double minAng;
		while (interruption && (Math.abs(x - odo.getX()) > CM_ERR || Math.abs(y - odo.getY()) > CM_ERR)) {
			minAng = (Math.atan2(y - odo.getY(), x - odo.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			turnTo(minAng, false);
			setSpeeds(FAST, FAST);
			setSpeeds(FAST, FAST);
			if (getFilteredData()< 30) {
				break;	
				}
		}
		rightMotor.setSpeed(0);
		leftMotor.setSpeed(0);
		isDone = true;
	}
	
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}
	
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odo.getAng();

		while (Math.abs(error) > DEG_ERR) {

			error = angle - this.odo.getAng();

			if (error < -180.0) {
				setSpeeds(-SLOW, SLOW);
			} else if (error < 0.0) {
				setSpeeds(SLOW, -SLOW);
			} else if (error > 180.0) {
				setSpeeds(SLOW, -SLOW);
			} else {
				setSpeeds(-SLOW, SLOW);
			}
		}

		if (stop) {
			setSpeeds(0, 0);
		}
	}

}
