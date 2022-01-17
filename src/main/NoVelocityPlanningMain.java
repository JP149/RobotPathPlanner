/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package main;

import java.awt.Color;
import java.awt.Point;

import javax.swing.JFrame;

import main.factory.SimulatorFactory;
import simulator.Robot;
import simulator.Simulator;
import simulator.env.obstacles.CircularObstacle;
import simulator.env.obstacles.RectangularObstacle;

public class NoVelocityPlanningMain {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		Simulator sim = SimulatorFactory.createNoVelocityPlannningSim();

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