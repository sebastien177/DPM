package ev3Navigation;

// Code taken from the teacher's slide
public interface UltrasonicController {
	
	public void processUSData(int distance);
	
	public int readUSDistance();
}
