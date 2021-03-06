package ev3Navigation;
/* Lab 3 Navigation
 * Group 7
 * Sebastien Arrese & Arnold Kokoroko
 */
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/* This class makes the robot navigates from 1 coordinate to another one taking its 
 * current position from Odometer
 */
public class Navigation extends Thread {
	double currentX, currentY, targetX, targetY; // Current Position / Target Position
	double currentTheta;//Current Angle in degrees
	Odometer odometer; 
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;

	// Class constants
	private static final long navigationPeriod = 25;
	private static final double wheelRadius = 2.145;
	private static final double wheelDistance = 15.25;
	private static final double errorThreshold = 5; 
	private static final int rotationSpeed = 180; 
	private static final int fwdSpeed = 350; 
	private static final int turnSpeed = 250; 


	// Coordinates Objects instantiation 
	Coordinate c0 = new Coordinate(0, 0); // Start 0,0
	Coordinate c1 = new Coordinate(60, 30);
	Coordinate c2 = new Coordinate(30, 30);
	Coordinate c3 = new Coordinate(30, 60);
	Coordinate c4 = new Coordinate(60, 0); // End 0,0
	
	//Constructor 
	public Navigation(Odometer odometer,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}
	
	public void run (){
		long updateStart, updateEnd;
		
		while (true) {
			updateStart = System.currentTimeMillis();
			//Get odometer values X Y and theta
			this.currentX = odometer.getX();
			this.currentY = odometer.getY();
			this.currentTheta = Math.toDegrees(odometer.getTheta()); //Converting Radians to Pi
			
			if (currentTheta > 180) {
				odometer.setTheta(currentTheta - 360);
			} else if (currentTheta < -180) {
				odometer.setTheta(currentTheta + 360);
			}
			
			//Different coordinate Scenarios + Setting Target points
			if (c4.getIsVisited()) {
				//End of navigation, motors stop
				leftMotor.stop();
				rightMotor.stop();
				break;
			} else if (c3.getIsVisited()) {
				setCoordinate(c4);
				c4.setIsVisited(currentX, currentY);
			} 
			else if (c2.getIsVisited()) {
				setCoordinate(c3);
				c3.setIsVisited(currentX, currentY);
			} 
			else if (c1.getIsVisited()) {
				setCoordinate(c2);
				c2.setIsVisited(currentX, currentY);
			} 
			else {
				//Initial scenario when at 0,0. Target becomes coordinate 1
				setCoordinate(c1);
				c1.setIsVisited(currentX, currentY);
			}
			
			// Once Target points are set, travel to them
			travelTo(this.targetX, this.targetY); 

			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < navigationPeriod) {
				try {
					Thread.sleep(navigationPeriod - (updateEnd - updateStart));
				} catch (InterruptedException e) {

				}
			}		
		}
	}
	//Target Setter Method
	public void setCoordinate (Coordinate c){
		this.targetX = c.getX();
		this.targetY = c.getY();
	}
	
	// Travel to Specific Target Point
	public void travelTo(double x, double y) {

		// Coordinate difference
		double deltaX = x - this.currentX;
		double deltaY = y - this.currentY;
		double targetTheta = Math.toDegrees(Math.atan2(deltaX, deltaY));
		double deltaTheta = targetTheta - this.currentTheta; 
		
		// If Angle difference is bigger than threshold, correct by Turning
		if (Math.abs(deltaTheta) > errorThreshold) {
			turnTo(targetTheta);
		} else {
		//If not, continue trajectory forward
			leftMotor.setSpeed(fwdSpeed);
			rightMotor.setSpeed(fwdSpeed);
			leftMotor.forward();
			rightMotor.forward();
		}
	}	

	// Turning method, adjusting itself
	public void turnTo(double theta) {
		double rotate = theta - this.currentTheta;	
		//Rotation occurs when robot is at a coordinate point
		if (c0.isAtPoint(currentX, currentY) || c1.isAtPoint(currentX, currentY) || c2.isAtPoint(currentX, currentY) || c3.isAtPoint(currentX, currentY) || c4.isAtPoint(currentX, currentY)){
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

		//ADJUST BUT NOT ROTATE towards left
		else if ( rotate > 0) {
			leftMotor.setSpeed(turnSpeed);
			rightMotor.setSpeed(fwdSpeed);
		}
		//ADJUST BUT NOT ROTATE towards right
		else if ( rotate < 0) {
			leftMotor.setSpeed(fwdSpeed);
			rightMotor.setSpeed(turnSpeed);
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

	
	
	

}
