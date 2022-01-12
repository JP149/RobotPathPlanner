/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package main;

import java.awt.Color;
import java.awt.Point;

import javax.swing.JFrame;

import simulator.Simulator;
import simulator.env.obstacles.CircularObstacle;
import simulator.env.obstacles.RectangularObstacle;
import simulator.env.obstacles.Robot;

public class NoVelocityPlanningMain {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		Simulator sim = new Simulator();
		sim.map.setCellSize(2);
		sim.map.setHeight(350);
		sim.map.setWidth(550);
		
		// define and add the robots
		Point r1s = new Point(30, 75);
		Point r2s = new Point(30, 150);
		Point r3s = new Point(30, 225);

		Point r1g = new Point(460, 115);
		Point r2g = new Point(460, 150);
		Point r3g = new Point(460, 185);

		// default sensing range =70
		double rVelocity = 10;
		Robot r1 = new Robot(r1s, r1g, rVelocity, sim.map);
		Robot r2 = new Robot(r2s, r2g, rVelocity, sim.map);
		Robot r3 = new Robot(r3s, r3g, rVelocity, sim.map);
		
		r1.Label = "R1";
		r2.Label = "R2";
		r3.Label = "R3";

		r1.Radius = 12;
		r2.Radius = 12;
		r3.Radius = 12;

		// set the colors
		r1.Color = Color.RED;
		r2.Color = Color.GREEN;
		r3.Color = Color.BLUE;

		sim.map.addObstacle(r1);
		sim.map.addObstacle(r2);
		sim.map.addObstacle(r3);

		// create and add the obstacles
		double staticVelocity = 0;
		RectangularObstacle ro1 = new RectangularObstacle(new Point(250, 75),
				200, 100, staticVelocity);
		RectangularObstacle ro2 = new RectangularObstacle(new Point(250, 225),
				200, 100, staticVelocity);
		CircularObstacle co1 = new CircularObstacle(new Point(400, 150), 27,
				staticVelocity);

		sim.map.addObstacle(ro1);
		sim.map.addObstacle(ro2);
		sim.map.addObstacle(co1);

		// sim.enableVelocityPlanning();// do initial velocity replanning
		// sim.velocityPlan();

		// create a window and display the simulator (which is a JPanel)
		JFrame frame = new JFrame("D* Lite Path Planning");
		frame.add(sim);
		frame.setSize(sim.map.getWidth(), sim.map.getHeight());
		frame.setVisible(true);
		sim.run();
	}
}