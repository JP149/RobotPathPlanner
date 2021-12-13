/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;
import java.util.List;

import javax.swing.JPanel;

import A_Star_Planner.Node;
import A_Star_Planner.VelocityPlanner_A_Star;

@SuppressWarnings("serial")
public class Simulator extends JPanel {
	public Map map;
	public boolean DrawGrid = false;
	VelocityPlanner_A_Star VelocityPlanner;
	List<Node> VelocityProfile;
	boolean velocityPlanningEnabled = false;
	boolean velocityProfiled = false;

	public Simulator() {
		ArrayList<Robot> Robots = new ArrayList<Robot>();
		ArrayList<EnvironmentObject> Obstacles = new ArrayList<EnvironmentObject>();
		map = new Map(Obstacles, Robots);
	}

	public void enableVelocityPlanning() {
		VelocityPlanner = new VelocityPlanner_A_Star(map.Robots);
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
		int size = map.Obstacles.size();
		for (int i = 0; i < size; i++)
			map.Obstacles.get(i).draw(g2d);

		// draw the robots
		size = map.Robots.size();
		for (int i = 0; i < size; i++)
			map.Robots.get(i).draw(g2d);
	}

	private void drawGrid(Graphics2D g2d) {
		if (map.CellSize <= 0)
			return;
		g2d.setColor(Color.BLACK);
		int paneWidth = this.getWidth();
		int paneHeight = this.getHeight();
		for (int i = 0; i < paneWidth; i += map.CellSize) {
			g2d.drawLine(i, 0, i, paneHeight);
		}
		for (int j = 0; j < paneHeight; j += map.CellSize) {
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
		for (int i = 0; i < map.Robots.size(); i++) {
			Robot r = map.Robots.get(i);
			r.checkReplan(map.Obstacles);
			r.step();
		}
	}

	private void stepVelocityPlan() {
		checkVelocityReplan();
		// step through velocity profile
		stepCurrentNodeTo(currentNodeIndex++);
	}

	private void checkVelocityReplan() {
		if (!plannedAtLeastOnce(map.Robots)) {
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
		for (int i = 0; i < map.Robots.size(); i++) {
			Robot r = map.Robots.get(i);
			boolean robotReplanned = r.checkReplan(map.Obstacles);
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
		for (Robot r : map.Robots)
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
		for (Robot r : map.Robots)
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