/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator;

import java.util.ArrayList;

import simulator.env.obstacles.EnvironmentObject;
import simulator.env.obstacles.Robot;

public class Map {
	private int width;
	private int height;
	private int cellSize = 10;
	private ArrayList<EnvironmentObject> obstacles;
	private ArrayList<Robot> robots;

	public Map() {
		
		this.obstacles = new ArrayList<EnvironmentObject>();
		this.robots = new ArrayList<Robot>();
	}
	
	public ArrayList<EnvironmentObject> getObstacles() {
		return obstacles;
	}
	
	public ArrayList<Robot> getRobots() {
		return robots;
	}
	
	public void addObstacle(EnvironmentObject obstacle)
	{
		if(obstacle instanceof Robot)
			this.robots.add((Robot) obstacle);
		else
			this.obstacles.add(obstacle);
	}
	
	public int getWidth() {
		return width;
	}
	
	public void setWidth(int width) {
		this.width = width;
	}
	
	public int getHeight() {
		return height;
	}
	
	public void setHeight(int height) {
		this.height = height;
	}
	
	public int getCellSize() {
		return cellSize;
	}
	
	public void setCellSize(int cellSize) {
		this.cellSize = cellSize;
	}
}