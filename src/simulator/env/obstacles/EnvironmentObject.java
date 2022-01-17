/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator.env.obstacles;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.util.ArrayList;

public abstract class EnvironmentObject implements IObstacle {
	// represents a simulation environment object
	public double Velocity = 0;
	public Point Location;
	public Color Color;
	public ArrayList<Color> KnownByRobotColors;

	public EnvironmentObject(double velocity, Point currentLocation) {
		this.Velocity = velocity;
		this.Location = currentLocation;
		this.Color = java.awt.Color.gray;
		KnownByRobotColors = new ArrayList<Color>();
	}
	
	public void registerRobot(Color robotColor)
	{
		if (!KnownByRobotColors.contains(robotColor))
			KnownByRobotColors.add(robotColor);
	}
	
	public boolean isInSensingRange(Point location, int sensingRange)
	{
		// assume point obstacle check if point is within the sensing range circle
		return location.distance(this.Location) <= sensingRange;
	}

	public abstract void draw(Graphics2D g);
}