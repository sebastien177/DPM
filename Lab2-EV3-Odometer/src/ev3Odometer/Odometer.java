/*
 * Odometer.java
 */

package ev3Odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;
	
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	//Wheel Radius
	double leftRadius = 2.1;
	double rightRadius = 2.1;
	double wheelDistance = 15.4;
	
	// Get Tacho counts for both wheels
	double oldTachoLeft = leftMotor.getTachoCount();
	double oldTachoRight = rightMotor.getTachoCount();
	
	//Tachometer values
	double currentTachoLeft, currentTachoRight, deltaTachoLeft, deltaTachoRight;
	//Angles
	double leftArc, rightArc,thetaChange, arcLengthTravelled;
	
	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer() {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		lock = new Object();
	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				theta = -0.7376;
				
				//Getting Tacho Count from each motor
				currentTachoLeft = leftMotor.getTachoCount();
				currentTachoRight = rightMotor.getTachoCount();
				
				//Delta Tacho Values
				deltaTachoLeft = currentTachoLeft - oldTachoLeft;
				deltaTachoRight = currentTachoRight - oldTachoRight;
				
				//Wheel Arcs
				leftArc = Math.toRadians(deltaTachoLeft)*leftRadius;
				rightArc = Math.toRadians(deltaTachoRight)*rightRadius;
				
				//Change in theta
				thetaChange = (rightArc - leftArc)/wheelDistance;
				
				//Distance travelled
				arcLengthTravelled = (rightArc+leftArc)/2;
				
				setX(x+arcLengthTravelled*Math.cos(Math.toRadians(theta + thetaChange/2)));
				setY(y+arcLengthTravelled*Math.sin(Math.toRadians(theta + thetaChange/2)));
				setTheta(theta + Math.toDegrees(thetaChange));
				
				//Update values
				oldTachoLeft = currentTachoLeft;
				oldTachoRight = currentTachoRight;
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