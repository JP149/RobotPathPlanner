/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator;

import java.awt.AlphaComposite;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

import dstarlite.DStarLite;
import dstarlite.State;
import simulator.env.obstacles.EnvironmentObject;

public class Robot extends EnvironmentObject {

	public int Radius;
	public int SensingRange = 70;
	public Point Start;
	public Point Goal;
	public String Label;
	
	public RobotPlanner robotPlanner = new RobotPlanner(false, true, false, 0, null, this);
	private RobotDrawer robotDrawer = new RobotDrawer(this);
	
	public Robot(Point Start, double velocity, Map map) {
		
		super(velocity, Start);
		this.Start = Start;
		this.robotPlanner.Map = map;
	}

	public Robot(Point Start, Point Goal, double velocity, Map map) {
		super(velocity, Start);
		this.Start = Start;
		this.Goal = Goal;
		this.robotPlanner.Map = map;
		
		//setup the planner
		robotPlanner.PathPlanner = new DStarLite();
		robotPlanner.currentState = new State();
		robotPlanner.currentState.x = Start.x / robotPlanner.Map.getCellSize();
		robotPlanner.currentState.y = Start.y / robotPlanner.Map.getCellSize();
		robotPlanner.PathPlanner.init(robotPlanner.currentState.x, robotPlanner.currentState.y, Goal.x / robotPlanner.Map.getCellSize(), Goal.y / robotPlanner.Map.getCellSize());

		this.robotPlanner.Path = new ArrayList<State>();
		this.robotPlanner.previousPaths = new ArrayList<List<State>>();
		robotPlanner.knownObstacles = new ArrayList<EnvironmentObject>();
	}

	public void updateLocation(Point newLocation) {
		this.Location = newLocation;
	}

	@Override
	public boolean isInObstacle(int x, int y) {// Robot can be a Circular
												// Obstacle
		return Math.pow(x - this.Location.x, 2) + Math.pow(y - this.Location.y, 2) <= Math.pow(Radius, 2);
	}

	@Override
	public void draw(Graphics2D g) {
		robotDrawer.draw(g);
	}
}