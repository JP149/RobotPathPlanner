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
import simulator.Simulator;
import simulator.env.obstacles.CircularObstacle;
import simulator.env.obstacles.RectangularObstacle;
import simulator.env.obstacles.Robot;

public class VelocityPlanningMain {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		Simulator sim = SimulatorFactory.createVelocityPlanningSim();
		
		sim.enableVelocityPlanning();// do initial velocity replanning
		sim.velocityPlan();
		
		// create a window and display the simulator (which is a JPanel)
		JFrame frame = new JFrame("Velocity Path Planning");
		frame.add(sim);
		frame.setSize(sim.map.getWidth(), sim.map.getHeight());
		frame.setVisible(true);
		// frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		sim.run();
	}
}