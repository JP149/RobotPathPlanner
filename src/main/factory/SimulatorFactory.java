package main.factory;

import java.awt.Color;
import java.awt.Point;

import formationsimulation.FormationSimulator;
import simulator.Simulator;
import simulator.env.obstacles.CircularObstacle;
import simulator.env.obstacles.RectangularObstacle;
import simulator.env.obstacles.Robot;

public class SimulatorFactory {
	
	public static Simulator createNoVelocityPlannningSim()
	{
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
		
		return sim;
	}
	
	public static Simulator createVelocityPlanningSim()
	{
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

		// //perform the planning
		sim.map.addObstacle(ro1);
		sim.map.addObstacle(ro2);
		sim.map.addObstacle(co1);
		
		return sim;
	}
	
	public static Simulator createFormationSim(double[] xyInterlaced)
	{
		FormationSimulator sim = new FormationSimulator();
		sim.map.setCellSize(1);
		sim.map.setHeight(350);
		sim.map.setWidth(350);
		// define and add the robots
		
		double[] d = xyInterlaced;
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
		
		return sim;
	}
}
