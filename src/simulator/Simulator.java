/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.List;

import javax.swing.JPanel;

import astarplanner.Node;
import simulator.env.obstacles.Robot;
import astarplanner.AStarVelocityPlanner;

@SuppressWarnings("serial")
public class Simulator extends JPanel {
	public Map map;
	public boolean DrawGrid = false;
	AStarVelocityPlanner VelocityPlanner;
	List<Node> VelocityProfile;
	boolean velocityPlanningEnabled = false;
	boolean velocityProfiled = false;

	public Simulator() {
		map = new Map();
	}

	public void enableVelocityPlanning() {
		VelocityPlanner = new AStarVelocityPlanner(map.getRobots());
		velocityPlanningEnabled = true;
		velocityProfiled = false;
	}

	public void velocityPlan() {
		if (velocityPlanningEnabled) {
			checkVelocityReplan();
		}
	}

	public int[] getCurrentLocation() {
		return currentNode == null ? null : currentNode.NodeInfo.Location;
	}

	public void disableVelocityPlanning() {
		VelocityPlanner = null;
		velocityPlanningEnabled = false;
		velocityProfiled = false;
	}

	@Override
	public void paint(Graphics g) {
		super.paint(g);
		Graphics2D g2d = (Graphics2D) g;
		g2d.setBackground(Color.white);
		// draw the grid
		if (DrawGrid)
			drawGrid(g2d);

		// draw the obstacles
		for (var obstacle : map.getObstacles())
			obstacle.draw(g2d);
		
		// draw the robots
		for (var robot : map.getRobots())
			robot.draw(g2d);
	}

	private void drawGrid(Graphics2D g2d) {
		int mapCellSize = map.getCellSize();
		if (mapCellSize<= 0)
			return;
		g2d.setColor(Color.BLACK);
		int paneWidth = this.getWidth();
		int paneHeight = this.getHeight();
		for (int i = 0; i < paneWidth; i += mapCellSize) {
			g2d.drawLine(i, 0, i, paneHeight);
		}
		for (int j = 0; j < paneHeight; j += mapCellSize) {
			g2d.drawLine(0, j, paneWidth, j);
		}
	}

	int currentNodeIndex;
	Node currentNode;

	private void step() {
		if (velocityPlanningEnabled)
			stepVelocityPlan();
		else
			stepRobot();
	}

	private void stepRobot() {
		for (Robot robot : map.getRobots()) {
			robot.checkReplan(map.getObstacles());
			robot.step();
		}
	}

	private void stepVelocityPlan() {
		checkVelocityReplan();
		// step through velocity profile
		stepCurrentNodeTo(currentNodeIndex++);
	}

	private void checkVelocityReplan() {
		if (!plannedAtLeastOnce(map.getRobots())) {
			// initial planning required
			planAllRobots();
			VelocityProfile = VelocityPlanner.replan();
			velocityProfiled = true;
			printVelocityProfile();
			currentNodeIndex = 0;
			stepCurrentNodeTo(currentNodeIndex);
			return;
		}
		// check if replanning is required
		
		for (Robot r : map.getRobots()) {
			boolean robotReplanned = r.checkReplan(map.getObstacles());
			if (robotReplanned) {// now Velocity replan
				VelocityProfile = VelocityPlanner.replan();
				velocityProfiled = true;
				printVelocityProfile();
				System.out.println("Robot " + r.Label + "\tReplanned");
				currentNodeIndex = 0;
				stepCurrentNodeTo(currentNodeIndex);
			}
		}
	}

	private void printVelocityProfile() {
		int i = 0;
		for (Node n : VelocityProfile) {
			System.out.print("Node " + (++i) + "\t");
			for (int l : n.NodeInfo.Location)
				System.out.print(l + "\t");
			System.out.println("");
		}
	}

	private void planAllRobots() {
		for (Robot r : map.getRobots())
			r.replan();
	}

	private boolean plannedAtLeastOnce(List<Robot> robots) {
		for (Robot r : robots)
			if (!r.hasPlannedOnce)
				return false;
		return true;
	}

	private void stepCurrentNodeTo(int nextIndex) {
		if (VelocityProfile == null || VelocityProfile.size() <= 0
				|| nextIndex >= VelocityProfile.size())// nothing to
														// do
			return;
		currentNode = VelocityProfile.get(nextIndex);
		for (Robot r : map.getRobots())
			r.stepTo(currentNode.NodeInfo.Location[VelocityPlanner.getIndex(r)]);
	}

	public void run() {
		while (true) {
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			this.step();
			this.repaint();
		}
	}
}