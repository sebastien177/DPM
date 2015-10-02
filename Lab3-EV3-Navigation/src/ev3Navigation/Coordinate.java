package ev3Navigation;
/* Lab 3 Navigation
 * Group 7
 * Sebastien Arrese & Arnold Kokoroko
 */

public class Coordinate {
	private double x,y;
	private boolean isVisited = false; // Set initially to false. Switches to True if visited.
	private final double errorMargin = 2; // near the target (measured in cm)
	
	public Coordinate (double x, double y){
		this.x = x;
		this.y = y;
	}
	
		// Getter Methods
		public double getX() {
			return this.x;
		}

		public double getY() {
			return this.y;
		}
		
		// Setters Methods
		public void setX(double x) {
			this.x = x;
		}

		public void setY(double y) {
			this.y = y;
		}
		
		// Getter Method to check if Coordinate has been visited
		public boolean getIsVisited() {
			return isVisited;
		}

		// Setter method to set visited
		public void setIsVisited(double x, double y) {
			isVisited = (Math.abs(x - getX()) < errorMargin)
					&& (Math.abs(y - getY()) < errorMargin);
		}

		// Check if robot is at the Coordinate point targeted
		public boolean isAtPoint(double x, double y) {
			return (Math.abs(x - getX()) < errorMargin)
					&& (Math.abs(y - getY()) < errorMargin);
		}

}
