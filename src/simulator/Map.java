/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator;

import java.util.ArrayList;

public class Map {
	public int Width;
	public int Height;
	public int CellSize = 10;
	public ArrayList<EnvironmentObject> Obstacles;
	public ArrayList<Robot> Robots;

	public Map(ArrayList<EnvironmentObject> Obstacles, ArrayList<Robot> Robots) {
		this.Obstacles = Obstacles;
		this.Robots = Robots;
		if (this.Obstacles == null)
			this.Obstacles = new ArrayList<EnvironmentObject>();
		if (this.Robots == null)
			this.Robots = new ArrayList<Robot>();
	}
}