package wallFollower;
import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController{
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh, motorMedium;
	private int distance;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	private int counter; //keeps track of recording
	
	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
							  int bandCenter, int bandwidth, int motorLow, int motorHigh, int motorMedium) {
		//Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.motorMedium = motorMedium;
		leftMotor.setSpeed(motorHigh);				// Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
	}
	
//	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		int distError =  this.distance - this.bandCenter;
		
		//If there is no error, we let the robot go straight ahead
		if(Math.abs(distError)<= this.bandwidth){
			this.leftMotor.setSpeed(this.motorMedium);
			this.rightMotor.setSpeed(this.motorMedium);
			this.leftMotor.forward();
			this.rightMotor.forward();
			//error is recalculated for future reference
			distError = this.distance - this.bandCenter;
			
		}
		
		//If the robot is too far from the wall
		else if (distError > 0 && distError <= 100){
			//If too far, robot will have to move to the left, right wheel moving faster
			this.leftMotor.setSpeed(this.motorLow);
			this.rightMotor.setSpeed(this.motorHigh-20);
			this.leftMotor.forward();
			this.rightMotor.forward();
			//error is recalculated for future reference
			distError = this.distance - this.bandCenter;
			
		}
		
		else if ( distError > 100){
			this.leftMotor.setSpeed(this.motorLow);
			this.rightMotor.setSpeed(this.motorLow + 50);
			this.leftMotor.forward();
			this.rightMotor.forward();
			//error is recalculated for future reference
			distError = this.distance - this.bandCenter;
			
		}
		
		//If the robot is too close to the wall 3 scenarios happen
		else if (distError < 0){
			//If its almost touching the wall, robot goes backward
			if (distError <= -25){
				this.leftMotor.setSpeed(this.motorMedium);
				this.rightMotor.setSpeed(this.motorMedium);
				this.leftMotor.backward();
				this.rightMotor.backward();				
			}
			
			// Once the robot goes backward straight, it turns at an angle to replace itself
			else if (distError <= -15 && distError > -25){
				this.leftMotor.setSpeed(this.motorHigh);
				this.rightMotor.setSpeed(this.motorHigh);
				this.leftMotor.forward();
				this.rightMotor.backward();				
			}
			//If close, but not super close, robot will have to move right away from the wall, left wheel moving faster
			else{
			this.leftMotor.setSpeed(this.motorHigh);
			this.rightMotor.setSpeed(this.motorLow);
			this.leftMotor.forward();
			this.rightMotor.forward();
			}
			//error is recalculated for future reference
			distError = this.distance - this.bandCenter;
		}

	}

//	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
