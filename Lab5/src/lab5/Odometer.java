package lab5;

/* Odometer class provided by TA's.
 * 
 * Lab 4: Localization
 * 
 * File: Odometer.java
 * 
 * ECSE-211: Design Principles and Methods
 * 
 * Students: Luke Soldano & Tuan-Anh Nguyen (Group 8)
 * 
 * Additional methods such as getX(), getY(), and getTheta() were added from Lab 3: Navigation
 * 
 * The odometer is passed through a robot so the robot will have access to the odometer used
 * throughout this lab.
 * 
 */



import lejos.*;
import lejos.utility.TimerListener;
import lejos.utility.Timer;

public class Odometer implements TimerListener {
	
	// Declaring class variables
	
	public static final int DEFAULT_PERIOD = 25;
	private TwoWheeledRobot robot;
	private Timer odometerTimer;
	private Navigation nav; // The navigator that will make the robot move to different coordinates
	// position data
	private Object lock;
	private double x, y, theta; // These are the x and y positions along with the angle the robot is pointed at
	private double[] oldDH, dDH;

	// Constructor passing a robot through this odometer so it will
	// make the odometer accessible while the robot is in motion
	
	public Odometer(TwoWheeledRobot robot) {
		this(robot, DEFAULT_PERIOD, false);
	}
	
	// Constructor for the odometer passing through a robot, the time at which the odometer updates
	// and a boolean to indicate to start the odometer right away
	
	public Odometer(TwoWheeledRobot robot, int period, boolean start) {
		// initialise variables
		this.robot = robot;
		this.nav = new Navigation(this);
		odometerTimer = new Timer(period, this);
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		oldDH = new double[2];
		dDH = new double[2];
		lock = new Object();

		// start the odometer immediately, if necessary
		if (start) {
			odometerTimer.start();
		}
	}

	// More constructors for the odometer passing through a robot and arguments
	// that will tell when and if the odometer should start right away or not
	
	public Odometer(TwoWheeledRobot robot, boolean start) {
		this(robot, DEFAULT_PERIOD, start);
	}

	public Odometer(TwoWheeledRobot robot, int period) {
		this(robot, period, false);
	}

	public void timedOut() {
		robot.getDisplacementAndHeading(dDH); // Getting the current heading and displacement of the robot
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];

		// update the position in a critical region
		synchronized (lock) {
			// Updating the angle on the odometer
			theta += dDH[1];
			theta = fixDegAngle(theta);
			
			// Updating x and y positions of the odometer using trigonometric equations
			x += dDH[0] * Math.sin(Math.toRadians(theta));
			y += dDH[0] * Math.cos(Math.toRadians(theta));
		}

		// Updating the old heading angle to compare it to the new one
		
		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}

	// Accessor that returns the positionL x-position, y-position, and current angle the robot is pointing to
	public void getPosition(double[] pos) {
		synchronized (lock) {
			pos[0] = x;
			pos[1] = y;
			pos[2] = theta;
		}
	}

	// Accessor that returns the current robot that is using this odometer
	public TwoWheeledRobot getTwoWheeledRobot() {
		return robot;
	}

	// Accessor that returns the current navigation the odometer is using
	public Navigation getNavigation() {
		return this.nav;
	}

	// Method that sets the position of the odometer based on where the robot currently is
	// Positions are: x-position, y-position, and current angle the robot is pointing to
	public void setPosition(double[] pos, boolean[] update) {
		synchronized (lock) {
			if (update[0]) {
				this.x = pos[0];
			}
			if (update[1]) {
				this.y = pos[1];
			}
			if (update[2]) {
				this.theta = pos[2];
			}
		}
	}

	// Static 'helper' methods
	
	// Altering the angle if the angle is negative since the odometer
	// resets if it goes into the "negatives". The odometer does not
	// display negative values of angles so it resets to 0 and decrements
	// from 360 degrees rather than using negative angles
	
	public static double fixDegAngle(double angle) {
		if (angle < 0.0) {
			angle = 360.0 + (angle % 360.0);
		}
		return angle % 360.0;
	}

	// Calculating the minimum angle from one point to another
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);

		if (d < 180.0) {
			return d;
		}
		else {
			return d - 360.0;
		}
	}

	// Accessor to get the current x-position of the robot
	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	// Accessor to get the current y-position of the robot
	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}
	
	// Accessor to get the current angle the robot is pointing to

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}
		return result;
	}

}