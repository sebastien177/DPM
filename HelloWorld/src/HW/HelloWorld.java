package HW;
import lejos.hardware.*;

public class HelloWorld {
	
	public static void main  (String [] args){
		double deltaX = 60;
		double deltaY = 30;
		double targetTheta = Math.toDegrees(Math.atan2(deltaY, deltaX));
		System.out.println(targetTheta);
		
	}

}
