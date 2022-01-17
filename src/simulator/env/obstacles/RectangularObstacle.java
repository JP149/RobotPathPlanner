/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator.env.obstacles;

import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.Point;

import dstarlite.DStarLite;
import simulator.Robot;

public class RectangularObstacle extends EnvironmentObject {
	public int XSpan;
	public int YSpan;

	public RectangularObstacle(Point currentLocation, int XSpan, int YSpan, double velocity) {
		super(velocity, currentLocation);
		this.XSpan = XSpan;
		this.YSpan = YSpan;
	}

	@Override
	public boolean isInObstacle(int x, int y) {
		return x - this.Location.x <= XSpan / 2.0 && y - this.Location.y <= YSpan / 2.0;
	}

	@Override
	public boolean isInSensingRange(Point location, int sensingRange) {
		// check if sensing range circle overlaps Rectangular Obstacle as
		// approximated by circle inscribed rectangle
		// approximation should suffice assuming the rectangle is smaller
		// than the sensing range
		double inscribedRectRadius = Math.sqrt(Math.pow(this.XSpan, 2) + Math.pow(this.YSpan, 2)) / 2.0;
		double centerDistance = location.distance(this.Location);
		return centerDistance < sensingRange + inscribedRectRadius;
	}

	@Override
	public void draw(Graphics2D g2d) {
		Graphics2D g = (Graphics2D) g2d.create();
		g.setColor(this.Color);
		g.fillRect(this.Location.x - this.XSpan / 2, this.Location.y - this.YSpan / 2, this.XSpan, this.YSpan);

		// draw the Obstacle's Robot Detected status - shows color of the robots
		// that have detected it

		int spacing = 2;
		g.setStroke(new BasicStroke(spacing));
		for (int j = 0; j < this.KnownByRobotColors.size(); j++) {
			java.awt.Color c = this.KnownByRobotColors.get(j);
			g.setColor(c);
			int XSpan = this.XSpan + 2 * ((j + 1) * spacing);// increases
																// spacing by 2
																// between each
																// color
			int YSpan = this.YSpan + 2 * ((j + 1) * spacing);
			g.drawRect(this.Location.x - XSpan / 2, this.Location.y - YSpan / 2, XSpan, YSpan);
		}
	}
	
	@Override
	public void updateCells(DStarLite dStarPathPlanner, int mapCellSize, Robot robot) {
		// expand the Obstacle by the robot size
		RectangularObstacle ro = new RectangularObstacle(this.Location, this.XSpan + 2 * robot.Radius,
				this.YSpan + 2 * robot.Radius, +this.Velocity);
		
		// update blocked cells in the rectangle
		int xOffset = ro.Location.x - ro.XSpan / 2;
		int yOffset = ro.Location.y - ro.YSpan / 2;

		for (int i = 0; i < ro.XSpan; i += mapCellSize)
			for (int j = 0; j < ro.YSpan; j += mapCellSize)
				if (ro.isInObstacle(xOffset + i, yOffset + j))
					dStarPathPlanner.updateCell((xOffset + i) / mapCellSize, (yOffset + j) / mapCellSize, -1);
	}
}