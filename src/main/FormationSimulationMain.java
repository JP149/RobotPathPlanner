/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package main;

import javax.swing.JFrame;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.FirstOrderIntegrator;
import org.apache.commons.math3.ode.nonstiff.DormandPrince853Integrator;
import formationsimulation.FormationTrack;
import formationsimulation.FormationStepHandler;
import main.factory.SimulatorFactory;
import simulator.Simulator;

public class FormationSimulationMain {

	public static void main(String[] args) {

		FirstOrderDifferentialEquations ode = createODE();

		RealMatrix Xd0 = MatrixUtils.createRealMatrix(new double[][] { { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 } });
		RealMatrix X0 = MatrixUtils
				.createRealMatrix(new double[][] { { -0.3, -0.4, 0.2, -0.1, 0.4, 0.1, -0.5, 0.2, 0.5, 0.2 } });
		RealMatrix E0 = X0.subtract(Xd0);

		double[] E0arr = E0.getData()[0];
		double[] E = new double[E0arr.length];

		final double[] xyInterlaced = new double[] { 0.1, 0.1, 0.1, 0.4, 0.4, 0.4, 0.4, 0.1, 0.25, 0.25 };

		Simulator sim = SimulatorFactory.createFormationSim(xyInterlaced);

		FormationStepHandler formationStepHandler = new FormationStepHandler(sim, xyInterlaced);

		FirstOrderIntegrator dp853 = new DormandPrince853Integrator(1.0e-8, 100, 1.0e-10, 1.0e-10);
		dp853.addStepHandler(formationStepHandler);

		displaySimWindow(sim);

		//animate the formation
		dp853.integrate(ode, 0.0, E0arr, 100.0, E);// solution from t=0 to t=100 seconds
		// E contains final error at Tfinal
	}

	static FirstOrderDifferentialEquations createODE() {

		RealMatrix Ai = MatrixUtils.createRealMatrix(new double[][] { { 0, 1 }, { 0, 0 } });
		RealMatrix Bi = MatrixUtils.createRealMatrix(new double[][] { { 1, 0 }, { 0, 1 } });
		double epsilon = 0.5;
		RealMatrix Fi = MatrixUtils.createRealMatrix(new double[][] { { 1, 2 }, { 1.5, 4 } });
		RealMatrix Pi = MatrixUtils.createRealMatrix(new double[][] { { 0.9102, 0.4142 }, { 0.4142, 1.2872 } });

		RealMatrix Lg = MatrixUtils.createRealMatrix(new double[][] { { 1, -1, 0, 0, 0 }, { -1, 3, -1, -1, 0 },
				{ 0, -1, 1, 0, 0 }, { 0, -1, 0, 1, 0 }, { 0, -1, 0, 0, 1 } });
		int N = 5;
		int m = 2;

		return new FormationTrack(N, m, Ai, Bi, Pi, Fi, epsilon, Lg);
	}
	
	static void displaySimWindow(Simulator sim)
	{
		// create a window and display the simulator (which is a JPanel)
		JFrame frame = new JFrame("Cooperative Formation Control");
		frame.add(sim);
		frame.setSize(sim.map.getWidth(), sim.map.getHeight());
		frame.setVisible(true);
	}
}
