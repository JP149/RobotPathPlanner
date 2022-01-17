package formationsimulation;

import java.util.ArrayList;

import org.apache.commons.math3.ode.sampling.StepInterpolator;
import org.apache.commons.math3.ode.sampling.StepHandler;

import dstarlite.State;
import simulator.Robot;
import simulator.Simulator;

public class FormationStepHandler implements StepHandler {
	
	private Simulator sim;
	private double[] xyInterlaced;
	
	public FormationStepHandler(Simulator sim, double[] xyInterlaced)
	{
		super();
		this.sim = sim;
		this.xyInterlaced = xyInterlaced;
	}
	
	public void init(double t0, double[] x, double t) {
	}

	public void handleStep(StepInterpolator interpolator, boolean isLast) {
		double t = interpolator.getCurrentTime();
		// t = Math.toRadians(t);
		// current numeric solution to error
		double[] E = interpolator.getInterpolatedState();

		
		ArrayList<Robot> robots =  sim.map.getRobots();
		double[] d = xyInterlaced;
		
		for (int i = 0; i < robots.size(); i++) {
			Robot r = robots.get(i);
			
			double x = Math.cos(t) + d[2 * i] * (-Math.sin(t))
					+ d[2 * i + 1] * Math.cos(t) + E[2 * i];
			double y = Math.sin(t) + d[2 * i] * Math.cos(t)
					+ d[2 * i + 1] * Math.sin(t) + E[2 * i + 1];
			r.robotPlanner.stepTo(new State((int) (x * 100) + 150,
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
}
