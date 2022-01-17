package simulator;

import java.awt.AlphaComposite;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;
import java.util.List;

import dstarlite.State;

public class RobotDrawer {
	
	private Robot robot;
	private RobotPlanner robotPlanner;
	
	public RobotDrawer(Robot robot)
	{
		this.robot = robot;
		this.robotPlanner = this.robot.robotPlanner;
	}
	
	public void draw(Graphics2D g2d) {
		Graphics2D g = (Graphics2D) g2d.create();

		drawSensingRange(g);

		drawOldPaths(g);

		drawNewPath(g);

		drawRobotAndLabel(g);
	}

	private void drawRobotAndLabel(Graphics2D g) {
		AlphaComposite ac = AlphaComposite.getInstance(AlphaComposite.SRC);
		g.setComposite(ac);
		
		g.setColor(robot.Color);
		g.fillOval(robot.Location.x - robot.Radius, robot.Location.y - robot.Radius, 2 * robot.Radius, 2 * robot.Radius);

		// draw its label
		FontMetrics fm = g.getFontMetrics();
		Rectangle2D rect = fm.getStringBounds(robot.Label, g);
		g.setColor(java.awt.Color.white);

		g.drawString(robot.Label, robot.Location.x - (float) rect.getWidth() / 2,
				robot.Location.y - (float) rect.getHeight() / 2 + fm.getAscent());
	}

	private void drawNewPath(Graphics2D g) {
		AlphaComposite ac = AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.65f);
		g.setComposite(ac);

		g.setColor(robot.Color.brighter());
		
		int mapCellSize = robotPlanner.Map.getCellSize();
		for (State s : robotPlanner.Path)
			g.fillRect(s.x * mapCellSize, s.y * mapCellSize, mapCellSize, mapCellSize);
	}

	private void drawOldPaths(Graphics2D g) {
		AlphaComposite ac = AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.12f);
		g.setComposite(ac);
		g.setColor(robot.Color.darker());
		int mapCellSize = robotPlanner.Map.getCellSize();
		for (List<State> path : robotPlanner.previousPaths)
			for (State s : path)
				g.fillRect(s.x * mapCellSize, s.y * mapCellSize, mapCellSize, mapCellSize);
	}

	private void drawSensingRange(Graphics2D g) {
		AlphaComposite ac = AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.2f);
		g.setComposite(ac);
		g.setColor(robot.Color);
		if (robotPlanner.DisplaySensingRange) {
			g.fillOval(robot.Location.x - robot.SensingRange, robot.Location.y - robot.SensingRange, 2 * robot.SensingRange,
					2 * robot.SensingRange);
		}
	}
}
