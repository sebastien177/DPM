package ev3ObjectRecognition;

import lejos.hardware.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

//This is the scanning class which makes the robot travel around the map and search for ANY objects
//If it sees an object, it will launch a new class.
public class Scan extends Thread{

	//Variables declaration
	private SampleProvider usSensor;
	private Odometer odo;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor usMotor;
	private static final int ROTATE_SPEED = 150;
	private static final double RIGHT_RADIUS = 2.145;
	private static final double LEFT_RADIUS = 2.145;
	private static final double WIDTH = 15.25;
	private static final double CM_ERR = 1.5;
	private boolean isFinished;
	private float[] usData;
	//The three next boolean will be explained inside the code
	private boolean coordinateNotReached = true;
	private boolean isDone = false;
	private boolean isObject = false;
	private int FAST = 150, SLOW = 100;
	private boolean interruption = false;
	final static double DEG_ERR = 3.0;

	public Scan (SampleProvider usSensor, float[] usData, SampleProvider colorSensor, float[] colorData, Odometer odo,  EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, EV3LargeRegulatedMotor usMotor){
		this.usSensor = usSensor;
		this.usData = usData;
		this.odo = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usMotor = usMotor;
	}

	//This constructor is used in USLocalizer to make the USsensor rotate
	public Scan (EV3LargeRegulatedMotor usMotor){
		this.usMotor = usMotor;
	}


	//This is the method launched by the thread, that makes the robot navigate around
	public void startRun(){
		isFinished = false;		

/*
		//might have to change
		odo.setPosition(new double[]{0.0,0.0,45.0}, new boolean[]{true,true,true});
		turn(-45);
*/
		turnTo(90, true);

		//Turn the USsensor after localization to scan the objects on the field on the right of the robot
		usMotor.setSpeed(70); 
		turnSensor(-90);

		//The boolean coordinateNotReached is to know if we reached the first coordinate yet
		while (coordinateNotReached){

			/*
			 * The isDone boolean is know when hte robot has to stop travelling 
			 * 2 cases : An object was detected or we reached the coordinate we set
			 */
			while (!isDone){
				travelTo(0, 75);

				//If we reached the wanted coordinate, then we didn't pick up any object... go to next coordinate
				if (isDone){
					coordinateNotReached = false;
				}

				//If there is an object inside 30cm on the right change the speed of the robot to go more slowly
				if (getFilteredData()< 30 && coordinateNotReached) {
					//While there is an object advance slowly
					FAST=30;
					interruption = true;
					while (getFilteredData()<45){
						travelTo(0,60);
					}
					interruption = false;
					//When there is no more object on the right stop the robot and put FAST to initial value for next iteration
					setSpeeds(0, 0);
					FAST=150;

					//set isObject and isDone to true, to go get the object
					isDone=true;
					isObject = true;
				}
			}
			isDone=false;

			/*If it was an object, turn the robot 90 degree and make it sensor look in front of the robot
			 * launch the BLockRecognition class
			 */
			if (isObject) {
				usMotor.setSpeed(50);
				turnSensor(90);
				turn(90);
				Lab5.br.startRun();
				//isFinished = true;
			}
			//isFinished = false;
			//Si les deux classes run en meme temps, use synchronized(lock)
			Sound.beepSequenceUp();

			//If we are back after BlockRecognition that means it wasn't a styrofoam
			//Make the robot continue it orginal travelling and scanning
			if(isObject){
				turn(-90); // turn CCW
				turnSensor(-90);
				leftMotor.forward();
				rightMotor.forward();
				Delay.msDelay(200);
			}
			isObject = false;
		}

		//Object was not found while going to first coordinate, go to next coordinate (60,60)
		coordinateNotReached = true;
		isDone = false;
		while (coordinateNotReached){

			/*
			 * The isDone boolean is know when hte robot has to stop travelling 
			 * 2 cases : An object was detected or we reached the coordinate we set
			 */
			while (!isDone){
				travelTo(75, 75);

				//If we reached the wanted coordinate, then we didn't pick up any object... go to next coordinate
				if (isDone){
					coordinateNotReached = false;
				}

				//If there is an object inside 30cm on the right change the speed of the robot to go more slowly
				if (getFilteredData()< 30 && coordinateNotReached) {
					//While there is an object advance slowly
					FAST=30;
					interruption = true;
					while (getFilteredData()<40){
						travelTo(60,60);
					}
					interruption = false;
					//When there is no more object on the right stop the robot and put FAST to initial value for next iteration
					setSpeeds(0, 0);
					FAST=150;

					//set isObject and isDone to true, to go get the object
					isDone=true;
					isObject = true;
				}
			}
			isDone=false;

			/*If it was an object, turn the robot 90 degree and make it sensor look in front of the robot
			 * launch the BLockRecognition class
			 */
			if (isObject) {
				usMotor.setSpeed(50);
				turnSensor(90);
				turn(90);
				Lab5.br.startRun();
				//isFinished = true;
			}
			//isFinished = false;
			//Si les deux classes run en meme temps, use synchronized(lock)
			Sound.beepSequenceUp();

			//If we are back after BlockRecognition that means it wasn't a styrofoam
			//Make the robot continue it orginal travelling and scanning
			if(isObject){
				turn(-90); // turn CCW
				turnSensor(-90);
				leftMotor.forward();
				rightMotor.forward();
				Delay.msDelay(200);
			}
			isObject = false;
		}
		isFinished = true;
	}


	// determines if robot is near target
	public boolean isNotThereYet(double x, double y) {
		return Math.abs(x - odo.getX()) > CM_ERR
				|| Math.abs(y - odo.getY()) > CM_ERR;
	}

	// Turn the motor attached to the USsensor
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



	//Return distance values from the UltrasonicSensor
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

	//First method to travel
	public void travelTo(double x, double y) {
		isDone  = false;
		double minAng;
		while (Math.abs(x - odo.getX()) > CM_ERR || Math.abs(y - odo.getY()) > CM_ERR) {
			minAng = (Math.atan2(y - odo.getY(), x - odo.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			turnTo(minAng, false);
			setSpeeds(FAST, FAST);
			setSpeeds(FAST, FAST);

			//While we are travelling if we see an object, stop the method
			if (getFilteredData()< 30) {
				return;
			}
			if(interruption){
				return;
			}
		}
		rightMotor.setSpeed(0);
		leftMotor.setSpeed(0);
		isDone = true;
	}

	//Call this second method multiple times to travel (it must be called inside a while loop)
	//This method doesn't have to be completed to execute another action
	public void travelTo2(double x, double y) {
		isDone  = false;
		double minAng;
		if (Math.abs(x - odo.getX()) > CM_ERR || Math.abs(y - odo.getY()) > CM_ERR) {
			minAng = (Math.atan2(y - odo.getY(), x - odo.getX())) * (180.0 / Math.PI);
			if (minAng < 0)
				minAng += 360.0;
			turnTo(minAng, false);
			setSpeeds(FAST, FAST);
			setSpeeds(FAST, FAST);
		} else{
			rightMotor.setSpeed(0);
			leftMotor.setSpeed(0);
			isDone = true;
		}
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
