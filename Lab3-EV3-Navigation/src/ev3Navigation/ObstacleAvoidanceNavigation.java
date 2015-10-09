package ev3Navigation;
/* Lab 3 Navigation
 * Group 7
 * Sebastien Arrese & Arnold Kokoroko
 */
import lejos.hardware.sensor.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.SampleProvider;
import lejos.hardware.Button;

//Made with our own code, the teacher's slide, the code from the first labs and the help of Jonna (TA)
public class ObstacleAvoidanceNavigation extends Thread implements UltrasonicController {

	private Object lock = new Object();;
	double currentX, currentY, targetX, targetY; // Current Position / Target Position
	double currentTheta;
	double obstacleTheta;
	boolean obstaclePresent = false;
	private static final int rotationSpeed = 180; 
	private static final int fwdSpeed = 360;  
	private static final int motorLow = 100;			// Speed of slower rotating wheel (deg/sec)
	private static final int motorMedium = 180;			// Speed of slower rotating wheel (deg/sec)
	private static final int motorHigh = 280;			// Speed of the faster rotating wheel (deg/seec)
	private int distance = 250;
	private final int bandCenter=30; 
	private boolean bangbang=true;
	Odometer odometer; 
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;

	// Class constants
	private static final long navigationPeriod= 25;
	private static final double wheelRadius = 2.1; // right wheel radius (cm)
	private static final double wheelDistance = 15.5; // wheel track (cm)
	private static final double errorThreshold = 1; // degrees off heading
	private static final int objectLimit = 30;


	// Coordinates for Navigation
	Coordinate c0 = new Coordinate(0, 0); 
	Coordinate c1 = new Coordinate(0, 60);
	Coordinate c2 = new Coordinate(60,0);

	public ObstacleAvoidanceNavigation(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor){
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	public void processUSData(int distance) {
		this.distance = distance;
		odometer.setDistance(distance);
	}


	public void run() {

		long updateStart, updateEnd;
		while (true) {

				updateStart = System.currentTimeMillis();
				this.currentX = odometer.getX();
				this.currentY = odometer.getY();
				this.currentTheta = Math.toDegrees(odometer.getTheta()); //Converting Radians to Pi

				if (currentTheta > 360) {
					odometer.setTheta(currentTheta - 360);
				} else if (currentTheta < -360) {
					odometer.setTheta(currentTheta + 360);
				}


				if (c2.getIsVisited()) {
					leftMotor.stop();
					rightMotor.stop();
					break;
				} else if (c1.getIsVisited()) {
					setCoordinate(c2);
					c2.setIsVisited(currentX, currentY);
				} 
				else {
					setCoordinate(c1);
					c1.setIsVisited(currentX, currentY);
				} 

				//Normal Navigation when no Object

				travelTo(targetX, targetY);

				updateEnd = System.currentTimeMillis();
				if (updateEnd - updateStart < navigationPeriod) {
					try {
						Thread.sleep(navigationPeriod - (updateEnd - updateStart));
					} catch (InterruptedException e) {
						// there is nothing to be done here because it is not
						// expected that the odometer will be interrupted by
						// another thread
					}
				}

			}
	}

	//Code taken from bangbang firstlab, makes the robot go around the obstacle
	public void avoidObstacle2(double x1, double y1){
		synchronized (lock) {
			while (distance < 30){
				int distError =  this.distance - this.bandCenter;	

				//If the robot is too close to the wall 3 scenarios happen
				if (distError < 0){
					//If its almost touching the wall, robot goes backward
					if (distError <= -25){
						this.leftMotor.setSpeed(this.motorMedium);
						this.rightMotor.setSpeed(this.motorMedium);
						this.leftMotor.backward();
						this.rightMotor.backward();	
					}

					// If not too close, turn left at high speed
					else if (distError <= -5 && distError > -25){
						this.leftMotor.setSpeed(this.motorLow);
						this.rightMotor.setSpeed(this.motorHigh);
						this.leftMotor.forward();
						this.rightMotor.forward();
					}
					
					//turn left slightly
					else if (distError <= 0){
						this.leftMotor.setSpeed(this.motorMedium);
						this.rightMotor.setSpeed(this.motorHigh);
						this.leftMotor.forward();
						this.rightMotor.forward();	
					}
					//else just go straight, we know there is a wall, but we're pretty far
					else{
						this.leftMotor.setSpeed(this.motorHigh);
						this.rightMotor.setSpeed(this.motorMedium);
						this.leftMotor.forward();
						this.rightMotor.forward();
						this.obstaclePresent = false;
					}					
					//error is recalculated for future reference
					distError = this.distance - this.bandCenter;
				}
			
		}

			
			/* If the code ahead (taken from bangbang 1st lab) wasn't executed yet,
			 * execute the following code which make the robot turn right 2 times
			 * and advance approx 10 cm
			 */
		if (bangbang){
			leftMotor.setSpeed(rotationSpeed);
			rightMotor.setSpeed(rotationSpeed);	
			
			leftMotor.rotate(convertAngle(wheelRadius, wheelDistance, -30), true);
			rightMotor.rotate(convertAngle(wheelRadius, wheelDistance, 30), false);
			
			this.leftMotor.setSpeed(this.motorHigh);
			this.rightMotor.setSpeed(this.motorHigh);
			this.leftMotor.forward();
			this.rightMotor.forward();
			
			try {
				   Thread.sleep(4000);
				}
				catch (InterruptedException e) {
				}
		}
		
		leftMotor.setSpeed(rotationSpeed);
		rightMotor.setSpeed(rotationSpeed);	
		
		leftMotor.rotate(convertAngle(wheelRadius, wheelDistance, 75), true);
		rightMotor.rotate(convertAngle(wheelRadius, wheelDistance, -75), false);
		
		this.leftMotor.setSpeed(this.motorHigh);
		this.rightMotor.setSpeed(this.motorHigh);
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		try {
			   Thread.sleep(3500);
			}
			catch (InterruptedException e) {
			}
		
		leftMotor.setSpeed(rotationSpeed);
		rightMotor.setSpeed(rotationSpeed);	
		leftMotor.rotate(convertAngle(wheelRadius, wheelDistance, 60), true);
		rightMotor.rotate(convertAngle(wheelRadius, wheelDistance,-60), false);
		
		this.leftMotor.setSpeed(this.motorHigh);
		this.rightMotor.setSpeed(this.motorHigh);
		this.leftMotor.forward();
		this.rightMotor.forward();
		
		try {
			   Thread.sleep(3000);
			}
			catch (InterruptedException e) {
			}
		
		leftMotor.stop();
		rightMotor.stop();
		}
		
		//If we encounter a new obstacle, perform a normal bangbang
		this.bangbang=false;
		
		//Now travel to our destination (obstacle is gone)
		travelTo(x1,y1);
	}


	public void setCoordinate (Coordinate c){
		this.targetX = c.getX();
		this.targetY = c.getY();
	}

	// Travel to absolute field location
	public void travelTo(double x, double y) {

			// Coordinate difference
			double deltaX = x - this.currentX;
			double deltaY = y - this.currentY;
			double targetTheta = Math.toDegrees(Math.atan2(deltaX, deltaY));
			double deltaTheta = targetTheta - this.currentTheta; 

			if( (this.distance < objectLimit) && bangbang ){
				avoidObstacle2(x,y);
			}
			// if the heading is off by more than acceptable error, we must correct
			if (Math.abs(deltaTheta) > errorThreshold) {
				turnTo(targetTheta);
			} else {

				leftMotor.setSpeed(fwdSpeed);
				rightMotor.setSpeed(fwdSpeed);
				leftMotor.forward();
				rightMotor.forward();
			}
	}	

	public void turnTo(double theta) {
		double rotate = theta - this.currentTheta;	
		//Meaning Complete rotation
		if (c0.isAtPoint(currentX, currentY) || c1.isAtPoint(currentX, currentY) || c2.isAtPoint(currentX, currentY)){
			if (rotate> 180){
				rotate -= 360;
			} else if (rotate < -180){
				rotate += 360;
			}
			leftMotor.setSpeed(rotationSpeed);
			rightMotor.setSpeed(rotationSpeed);	
			//Rotation starts
			leftMotor.rotate(convertAngle(wheelRadius, wheelDistance, rotate), true);
			rightMotor.rotate(-convertAngle(wheelRadius, wheelDistance, rotate), false);		
		}
		
		//This is the turnTo after bangbang was called
		else {
			leftMotor.setSpeed(rotationSpeed);
			rightMotor.setSpeed(rotationSpeed);	
			leftMotor.rotate(convertAngle(wheelRadius, wheelDistance, rotate), true);
			rightMotor.rotate(-convertAngle(wheelRadius, wheelDistance, rotate), false);	
		}

	}


	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	public boolean isNavigating() {
		return false;
	}

	public int readUSDistance() {
		return this.distance;
	}

}