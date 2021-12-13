/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package VelocityPlanningSimulation;

import java.awt.Color;
import java.awt.Point;

import javax.swing.JFrame;

import simulator.CircularObstacle;
import simulator.RectangularObstacle;
import simulator.Robot;
import simulator.Simulator;

public class NoVelocityPlanningMain {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		Simulator sim = new Simulator();
		sim.map.CellSize = 2;
		sim.map.Height = 350;
		sim.map.Width = 550;
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

		sim.map.Robots.add(r1);
		sim.map.Robots.add(r2);
		sim.map.Robots.add(r3);

		// create and add the obstacles
		double staticVelocity = 0;
		RectangularObstacle ro1 = new RectangularObstacle(new Point(250, 75),
				200, 100, staticVelocity);
		RectangularObstacle ro2 = new RectangularObstacle(new Point(250, 225),
				200, 100, staticVelocity);
		CircularObstacle co1 = new CircularObstacle(new Point(400, 150), 27,
				staticVelocity);

		// // (Default color is gray)
		// //ro1 known static obstacle
		// r1.processKnownObject(ro1);
		// r2.processKnownObject(ro1);
		// r3.processKnownObject(ro1);
		//
		// //ro2 known static obstacle
		// r1.processKnownObject(ro2);
		// r2.processKnownObject(ro2);
		// r3.processKnownObject(ro2);
		//
		// //co1 unknown

		sim.map.Obstacles.add(ro1);
		sim.map.Obstacles.add(ro2);
		sim.map.Obstacles.add(co1);

		// sim.enableVelocityPlanning();// do initial velocity replanning
		// sim.velocityPlan();

		// create a window and display the simulator (which is a JPanel)
		JFrame frame = new JFrame("D* Lite Path Planning");
		frame.add(sim);
		frame.setSize(sim.map.Width, sim.map.Height);
		frame.setVisible(true);
		// frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		sim.run();
	}
}