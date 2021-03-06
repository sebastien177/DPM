package wallFollower;
import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController{
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh;
	private int distance;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	
	private int counter; //keeps track of recording
	
	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
							  int bandCenter, int bandwidth, int motorLow, int motorHigh) {
		//Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(motorHigh);				// Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
		counter = 0;
	}
	
//	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in (BANG-BANG style)
		double distError =  this.distance - this.bandCenter;
		
		//If there is no error, we let the robot go straight ahead
		if(Math.abs(distError)<= this.bandwidth){
			this.leftMotor.setSpeed(this.motorLow);
			this.rightMotor.setSpeed(this.motorLow);
			this.leftMotor.forward();
			this.rightMotor.forward();
			//error is recalculated for future reference
			distError = this.distance - this.bandCenter;
			
		}
		
		//If the robot is too far from the wall
		else if (Math.abs(distError) > 0 && Math.abs(distError) < 200){
			counter = 0;
			//If too far, robot will have to move to the right, left wheel moving faster
			this.leftMotor.setSpeed(this.motorHigh);
			this.rightMotor.setSpeed(this.motorLow);
			this.leftMotor.forward();
			this.rightMotor.forward();
			//error is recalculated for future reference
			distError = this.distance - this.bandCenter;
			
		}
		
		//If the robot is too close to the wall
		else if (Math.abs(distError) < 0){
			counter = 0;
			//If too far, robot will have to move left away from the wall, right wheel moving faster
			this.leftMotor.setSpeed(this.motorLow);
			this.rightMotor.setSpeed(this.motorHigh);
			this.leftMotor.forward();
			this.rightMotor.forward();
			distError = this.distance - this.bandCenter;
		}
		
		// TODO: STILL NEED TO EVALUATE SCENARIO WHERE THE ROBOT HAS NOTHING AROUND IT
		
		


	}

//	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
