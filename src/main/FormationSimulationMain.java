/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package main;

import java.awt.Color;
import java.awt.Point;
import java.util.ArrayList;

import javax.swing.JFrame;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import org.apache.commons.math3.ode.sampling.StepHandler;
import org.apache.commons.math3.ode.sampling.StepInterpolator;

import dstarlite.State;
import formationsimulation.Def_Track;
import formationsimulation.FormationSimulator;
import simulator.env.obstacles.Robot;

public class FormationSimulationMain {

	public static void main(String[] args) {
		RealMatrix Ai = MatrixUtils.createRealMatrix(new double[][] { { 0, 1 },
				{ 0, 0 } });
		RealMatrix Bi = MatrixUtils.createRealMatrix(new double[][] { { 1, 0 },
				{ 0, 1 } });
		double epsilon = 0.5;
		RealMatrix Fi = MatrixUtils.createRealMatrix(new double[][] { { 1, 2 },
				{ 1.5, 4 } });
		RealMatrix Pi = MatrixUtils.createRealMatrix(new double[][] {
				{ 0.9102, 0.4142 }, { 0.4142, 1.2872 } });
		// RealMatrix tt = Pi.multiply(Bi).multiply(Fi);
		RealMatrix Lg = MatrixUtils.createRealMatrix(new double[][] {
				{ 1, -1, 0, 0, 0 }, { -1, 3, -1, -1, 0 }, { 0, -1, 1, 0, 0 },
				{ 0, -1, 0, 1, 0 }, { 0, -1, 0, 0, 1 } });
		RealMatrix Xd0 = MatrixUtils.createRealMatrix(new double[][] { { 1, 0,
				1, 0, 1, 0, 1, 0, 1, 0 } });
		RealMatrix X0 = MatrixUtils.createRealMatrix(new double[][] { { -0.3,
				-0.4, 0.2, -0.1, 0.4, 0.1, -0.5, 0.2, 0.5, 0.2 } });
		RealMatrix E0 = X0.subtract(Xd0);
		int N = 5;
		int m = 2;

		FirstOrderIntegrator dp853 = new DormandPrince853Integrator(1.0e-8,
				100, 1.0e-10, 1.0e-10);
		FirstOrderDifferentialEquations ode = new Def_Track(N, m, Ai, Bi, Pi,
				Fi, epsilon, Lg);
		double[] E0arr = E0.getData()[0];
		double[] E = new double[E0arr.length];
		final double[] d = new double[] { 0.1, 0.1, 0.1, 0.4, 0.4, 0.4, 0.4,
				0.1, 0.25, 0.25 };

		final FormationSimulator sim = new FormationSimulator();
		sim.map.setCellSize(1);
		sim.map.setHeight(350);
		sim.map.setWidth(350);
		// define and add the robots

		// map positions -1.5, 1.5 to 0,300 for drawing
		Point r1s = new Point((int) (d[0] * 100) + 150,
				(int) (d[1] * 100) + 150);
		Point r2s = new Point((int) (d[2] * 100) + 150,
				(int) (d[3] * 100) + 150);
		Point r3s = new Point((int) (d[4] * 100) + 150,
				(int) (d[5] * 100) + 150);
		Point r4s = new Point((int) (d[6] * 100) + 150,
				(int) (d[7] * 100) + 150);
		Point r5s = new Point((int) (d[8] * 100) + 150,
				(int) (d[9] * 100) + 150);
		double rVelocity = 0;
		Robot r1 = new Robot(r1s, rVelocity, sim.map);
		Robot r2 = new Robot(r2s, rVelocity, sim.map);
		Robot r3 = new Robot(r3s, rVelocity, sim.map);
		Robot r4 = new Robot(r4s, rVelocity, sim.map);
		Robot r5 = new Robot(r5s, rVelocity, sim.map);
		r1.Label = "R1";
		r2.Label = "R2";
		r3.Label = "R3";
		r4.Label = "R4";
		r5.Label = "R5";

		// set the colors
		r1.Color = Color.RED;
		r2.Color = Color.GREEN;
		r3.Color = Color.BLUE;
		r4.Color = Color.BLACK;
		r5.Color = Color.CYAN;

		sim.map.addObstacle(r1);
		sim.map.addObstacle(r2);
		sim.map.addObstacle(r3);
		sim.map.addObstacle(r4);
		sim.map.addObstacle(r5);
		
		for (Robot r : sim.map.getRobots()) {
			r.DisplaySensingRange = false;
			r.Radius = 10;
		}
		// updates the robot state each integration step
		StepHandler stepHandler = new StepHandler() {
			public void init(double t0, double[] x, double t) {
			}

			public void handleStep(StepInterpolator interpolator, boolean isLast) {
				double t = interpolator.getCurrentTime();
				// t = Math.toRadians(t);
				// current numeric solution to error
				double[] E = interpolator.getInterpolatedState();

				
				ArrayList<Robot> robots =  sim.map.getRobots();
				
				for (int i = 0; i < robots.size(); i++) {
					Robot r = robots.get(i);
					
					double x = Math.cos(t) + d[2 * i] * (-Math.sin(t))
							+ d[2 * i + 1] * Math.cos(t) + E[2 * i];
					double y = Math.sin(t) + d[2 * i] * Math.cos(t)
							+ d[2 * i + 1] * Math.sin(t) + E[2 * i + 1];
					r.stepTo(new State((int) (x * 100) + 150,
							(int) (y * 100) + 150, null));// map -1.5,1.5 to
															// 0,300
				}
				sim.repaint();
				try {
					Thread.sleep(150);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		};

		dp853.addStepHandler(stepHandler);

		// create a window and display the simulator (which is a JPanel)
		JFrame frame = new JFrame("Cooperative Formation Control");
		frame.add(sim);
		frame.setSize(sim.map.getWidth(), sim.map.getHeight());
		frame.setVisible(true);
		// frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		dp853.integrate(ode, 0.0, E0arr, 100.0, E);// solution from t=0 to t=100
													// seconds
		// E contains final error at Tfinal
	}
}
