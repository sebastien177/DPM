package wallFollower;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {
	
	private final int bandCenter, bandwidth;
	private final int motorStraight = 200, FILTER_OUT = 20;
	private int maxSpeedLimit = 300;
	private int minSpeedLimit = 100;
	private int newHighMotor, newLowMotor;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int distance;
	private int filterControl;
	private int distError;
	
	public PController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
					   int bandCenter, int bandwidth) {
		//Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(motorStraight);					// Initalize motor rolling forward
		rightMotor.setSpeed(motorStraight);
		leftMotor.forward();
		rightMotor.forward();
		filterControl = 0;
	}
	
	//@Override
	public void processUSData(int distance) {
		 this.distError =  this.distance - this.bandCenter;
		
		// rudimentary filter - toss out invalid samples corresponding to null signal.
		// (n.b. this was not included in the Bang-bang controller, but easily could have).
		//
		if (distance == 255 && filterControl < FILTER_OUT) {
			// bad value, do not set the distance var, however do increment the filter value
			filterControl ++;
		} else if (distance == 255){
			// true 255, therefore set distance to 255
			this.distance = distance;
		}
				
		else {
			// distance went below 255, therefore reset everything.
			filterControl = 0;
			this.distance = distance;
		}
		
		//If there is no error, we let the robot go straight ahead
		if(Math.abs(distError)<= this.bandwidth){
			this.leftMotor.setSpeed(this.motorStraight);
			this.rightMotor.setSpeed(this.motorStraight);
			this.leftMotor.forward();
			this.rightMotor.forward();
			
			filterControl = 0;
			//error is recalculated for future reference
			this.distError = this.distance - this.bandCenter;
			
		}
		//If the robot is too far from the wall
		//If too far, robot will have to move to the left, right wheel moving faster
		else if (distError > 0 && distError < 200){
			turn(this.rightMotor, this.leftMotor);
		}
		
		else if(distError >= 200){
			turnCornerConvex();
		}
		
		//If the robot is too close to the wall 3 scenarios happen
		else if (distError < 0){
			//If its almost touching the wall, robot goes backward
			if (distError <= -26){
				this.leftMotor.setSpeed(this.motorStraight);
				this.rightMotor.setSpeed(this.motorStraight);
				this.leftMotor.backward();
				this.rightMotor.backward();				
			}
			// Once the robot goes backward straight, it turns at an angle to replace itself
			else if (distError <= -15 && distError > -26){
				this.leftMotor.setSpeed(this.motorStraight+70);
				this.rightMotor.setSpeed(this.motorStraight+70);
				this.leftMotor.forward();
				this.rightMotor.backward();				
			}
			//If close, but not super close, robot will have to move right away from the wall, left wheel moving faster
			else{
			turn(this.leftMotor, this.rightMotor);
			}
			
		}
		

	}
	// Turn method following P style related to Error. Called in different scenarios.
	public void turn (EV3LargeRegulatedMotor motorToSpeed,EV3LargeRegulatedMotor motorToSlow){
		//Setting new speeds related to error
		newHighMotor = (this.motorStraight + Math.abs(this.distError)*6);
		newLowMotor = (this.motorStraight - Math.abs(this.distError)*6);
		
		//Scenario Max limit is reached
		if (newHighMotor > this.maxSpeedLimit){
			motorToSpeed.setSpeed(this.maxSpeedLimit);
		}
		// New High speed is set
		else{
			motorToSpeed.setSpeed(newHighMotor);			
		}
		
		//Scenario Min limit is reached
		if (newLowMotor < this.minSpeedLimit){
			motorToSlow.setSpeed(this.minSpeedLimit);
		}
		//New Slow speed is set
		else{
			motorToSlow.setSpeed(newLowMotor);			
		}
		
		motorToSpeed.forward();
		motorToSlow.forward();
		//error is recalculated for future reference
		this.distError = this.distance - this.bandCenter;
		
	}
	
	//Wider Turns
	public void turnCornerConvex (){
		rightMotor.setSpeed(motorStraight+50);
		leftMotor.setSpeed(motorStraight);
		rightMotor.forward();
		leftMotor.forward();
	}

	
	//@Override
	public int readUSDistance() {
		return this.distance;
	}

}
