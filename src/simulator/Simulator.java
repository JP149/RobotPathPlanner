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
import astarplanner.AStarVelocityPlanner;

@SuppressWarnings("serial")
public class Simulator extends JPanel {
	public Map map;
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
		if (velocityPlanningEnabled)
			checkVelocityReplan();
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
		
		// draw the obstacles
		for (var obstacle : map.getObstacles())
			obstacle.draw(g2d);
		
		// draw the robots
		for (var robot : map.getRobots())
			robot.draw(g2d);
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
			robot.robotPlanner.checkReplan(map.getObstacles());
			robot.robotPlanner.step();
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
			currentNodeIndex = 0;
			stepCurrentNodeTo(currentNodeIndex);
			return;
		}
		// check if replanning is required
		
		for (Robot r : map.getRobots()) {
			boolean robotReplanned = r.robotPlanner.checkReplan(map.getObstacles());
			if (robotReplanned) {// now Velocity replan
				VelocityProfile = VelocityPlanner.replan();
				velocityProfiled = true;
				System.out.println("Robot " + r.Label + "\tReplanned");
				currentNodeIndex = 0;
				stepCurrentNodeTo(currentNodeIndex);
			}
		}
	}

	private void planAllRobots() {
		for (Robot r : map.getRobots())
			r.robotPlanner.replan();
	}

	private boolean plannedAtLeastOnce(List<Robot> robots) {
		for (Robot r : robots)
			if (!r.robotPlanner.hasPlannedOnce)
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
			r.robotPlanner.stepTo(currentNode.NodeInfo.Location[VelocityPlanner.getIndex(r)]);
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