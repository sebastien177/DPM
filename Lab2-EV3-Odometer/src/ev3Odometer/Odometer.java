/*
 * Odometer.java
 */

package ev3Odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	//Robot position
	private double x, y, theta;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;

	
	//Robot Constants
	private static final double wheelRadius = 2.145;
	private static final double wheelDistance = 15.25;
	
	//Tacho values for Odometer
	private double tachoLeftLast = 0;
	private double tachoRightLast = 0;
	

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		//theta = -0.7376;
		lock = new Object();
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
		double displacement; // Magnitude
		double angleChange; // Delta Theta_n in equations

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here
			
			// Getting tacho counts for left and right wheel
			double tachoLeftNow = leftMotor.getTachoCount();
			double tachoRightNow = rightMotor.getTachoCount();
						
			// Calculating the change in rotation of each wheel
			double deltaTachoLeft = tachoLeftNow - tachoLeftLast;
			double deltaTachoRight = tachoRightNow - tachoRightLast;
			
			//Update for next cycle
			this.tachoLeftLast = tachoLeftNow;
			this.tachoRightLast = tachoRightNow;
			
			//Wheel Distance
			
			double leftWheelDistance = this.wheelRadius * deltaTachoLeft * (Math.PI/180);
			double rightWheelDistance = this.wheelRadius * deltaTachoRight * (Math.PI/180);
			
			//Robot Changes
			displacement =  (leftWheelDistance+rightWheelDistance)/2;
			angleChange = (leftWheelDistance-rightWheelDistance)/wheelDistance;

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!

				//theta = -0.7376;
				setTheta(theta + angleChange);
				double deltaX = displacement * Math.sin(this.theta);
				double deltaY = displacement * Math.cos(this.theta);
				//Current y and x get updated
				setX(this.x + deltaX);
				setY(this.y + deltaY);

				
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
}