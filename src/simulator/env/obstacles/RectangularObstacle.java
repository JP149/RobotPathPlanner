/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator.env.obstacles;

import java.awt.BasicStroke;
import java.awt.Graphics2D;
import java.awt.Point;

public class RectangularObstacle extends EnvironmentObject {
	public int XSpan;
	public int YSpan;

	public RectangularObstacle(Point currentLocation, int XSpan, int YSpan,
			double velocity) {
		super(velocity, currentLocation);
		this.XSpan = XSpan;
		this.YSpan = YSpan;
	}

	@Override
	public boolean isInObstacle(int x, int y) {
		return x - this.Location.x <= XSpan / 2.0
				&& y - this.Location.y <= YSpan / 2.0;
	}

	@Override
	public void draw(Graphics2D g2d) {
		Graphics2D g = (Graphics2D) g2d.create();
		g.setColor(this.Color);
		g.fillRect(this.Location.x - this.XSpan / 2, this.Location.y
				- this.YSpan / 2, this.XSpan, this.YSpan);

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
			g.drawRect(this.Location.x - XSpan / 2,
					this.Location.y - YSpan / 2, XSpan, YSpan);
		}
	}
}