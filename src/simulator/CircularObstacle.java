/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator;

import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.Point;

public class CircularObstacle extends EnvironmentObject {

	public int Radius;

	public CircularObstacle(Point currentLocation, int Radius, double velocity) {
		super(velocity, currentLocation);
		this.Radius = Radius;
	}

	@Override
	public boolean isInObstacle(int x, int y) {
		return Math.pow(x - this.Location.x, 2)
				+ Math.pow(y - this.Location.y, 2) <= Math.pow(Radius, 2);
	}

	@Override
	public void draw(Graphics2D g2d) {
		Graphics2D g = (Graphics2D) g2d.create();
		g.setColor(this.Color);
		g.fillOval(this.Location.x - this.Radius,
				this.Location.y - this.Radius, 2 * this.Radius, 2 * this.Radius);
		// draw the Obstacle's Robot Detected status - shows color of the robots
		// that have detected it
		int spacing = 2;
		g.setStroke(new BasicStroke(spacing));
		for (int j = 0; j < this.KnownByRobotColors.size(); j++) {
			java.awt.Color c = this.KnownByRobotColors.get(j);
			g.setColor(c);
			int Radius = this.Radius + spacing * (j + 1);// increase spacing
															// between each
			// color by 4
			g.drawOval(this.Location.x - Radius, this.Location.y - Radius,
					2 * Radius, 2 * Radius);
		}
	}
}
