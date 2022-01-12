/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package simulator.env.obstacles;

import java.awt.AlphaComposite;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.List;

import dstarlite.DStarLite;
import dstarlite.State;
import simulator.Map;

public class Robot extends EnvironmentObject {
	
	public int Radius;
	public int SensingRange = 70;
	public Point Start;
	public Point Goal;
	public List<State> Path;
	public List<State> RemainingPath;
	public ArrayList<List<State>> previousPaths;
	public String Label;
	public boolean IsReplanning = false;
	public ArrayList<EnvironmentObject> knownObstacles;
	DStarLite PathPlanner;
	
	public Map Map;
	public boolean DisplaySensingRange = true;
	public boolean hasPlannedOnce = false;

	public Robot(Point Start, double velocity, Map map) {// for formation
															// control
															// simulation
		super(velocity, Start);
		this.Start = Start;
		this.Map = map;
		currentState = new State();
		this.Path = new ArrayList<State>();
		this.previousPaths = new ArrayList<List<State>>();
		knownObstacles = new ArrayList<EnvironmentObject>();
	}

	public Robot(Point Start, Point Goal, double velocity, Map map) {
		super(velocity, Start);
		this.Start = Start;
		this.Goal = Goal;
		this.Map = map;
		PathPlanner = new DStarLite();
		currentState = new State();
		currentState.x = Start.x / Map.getCellSize();
		currentState.y = Start.y / Map.getCellSize();
		PathPlanner.init(currentState.x, currentState.y, Goal.x / Map.getCellSize(),
				Goal.y / Map.getCellSize());

		this.Path = new ArrayList<State>();
		this.previousPaths = new ArrayList<List<State>>();
		knownObstacles = new ArrayList<EnvironmentObject>();
	}

	public void updateLocation(Point newLocation) {
		this.Location = newLocation;
	}

	// range sensor implementation
	public boolean inSensingRange(EnvironmentObject envObject) {
		if (envObject instanceof CircularObstacle) {
			// check if sensing range circle overlaps Circle Obstacle
			CircularObstacle co = (CircularObstacle) envObject;
			double centerDistance = this.Location.distance(co.Location);
			return centerDistance < this.SensingRange + co.Radius;
		} else if (envObject instanceof RectangularObstacle) {
			// check if sensing range circle overlaps Rectangular Obstacle as
			// approximated by circle inscribed rectangle
			// approximation should suffice assuming the rectangle is smaller
			// than the sensing range
			RectangularObstacle ro = (RectangularObstacle) envObject;
			double inscribedRectRadius = Math.sqrt(Math.pow(ro.XSpan, 2)
					+ Math.pow(ro.YSpan, 2)) / 2.0;
			double centerDistance = this.Location.distance(ro.Location);
			return centerDistance < this.SensingRange + inscribedRectRadius;
		} else
			// assume point obstacle check if point is within the sensing range
			// circle
			return this.Location.distance(envObject.Location) <= this.SensingRange;
	}

	public boolean checkReplan(ArrayList<EnvironmentObject> environmentObjects) {
		boolean replanNecessary = !hasPlannedOnce
				|| ((pathIndex < RemainingPath.size()) && (processEnvironment(environmentObjects)));
		if (replanNecessary)
			replan();
		return replanNecessary;
	}

	public void replan() {
		if (!hasPlannedOnce)
			hasPlannedOnce = true;
		IsReplanning = true;// trigger drawing of replan status
		long elapsed = System.currentTimeMillis();
		PathPlanner.updateStart(currentState.x, currentState.y);
		PathPlanner.replan();
		if (this.Path.size() > 1)
			previousPaths.add(this.Path);
		this.Path = PathPlanner.getPath();
		this.RemainingPath = this.Path;
		pathIndex = 0;

		elapsed -= System.currentTimeMillis();
		System.out.println("D*Lite Replan: " + elapsed + "ms");
		IsReplanning = false;
	}

	public boolean processEnvironment(
			ArrayList<EnvironmentObject> environmentObjects) {
		// detect environment changes
		// process newly detected obstacles
		// check for new objects within the robot sensing range
		boolean replanNecessary = false;
		for (int i = 0; i < environmentObjects.size(); i++) {
			EnvironmentObject eo = environmentObjects.get(i);
			if (!knownObstacles.contains(eo) && inSensingRange(eo)) {
				replanNecessary = processKnownObject(eo);
			}
		}
		return replanNecessary;
	}

	public boolean processKnownObject(EnvironmentObject eo) {
		if (!knownObstacles.contains(eo)) {
			knownObstacles.add(eo);// obstacle is now known to the robot -
									// assumes obstacles are stationary
			if (!eo.KnownByRobotColors.contains(this.Color))
				eo.KnownByRobotColors.add(this.Color);// update color for
														// display
														// to show that the
														// robot has
														// detected it
			updateCells(eo);// update the corresponding
							// obstacle cells
			return true;// re-plan necessary
		}
		return false;// re-plan unnecessary - object is already known or not in
						// sensing range
	}

	private void updateCells(EnvironmentObject eo) {
		int mapCellSize = Map.getCellSize();
		
		if (eo instanceof RectangularObstacle) {
			RectangularObstacle ro = (RectangularObstacle) eo;
			// expand the Obstacle by the robot size
			ro = new RectangularObstacle(ro.Location, ro.XSpan + 2
					* this.Radius, ro.YSpan + 2 * this.Radius, +ro.Velocity);
			// update blocked cells in the rectangle
			int xOffset = ro.Location.x - ro.XSpan / 2;
			int yOffset = ro.Location.y - ro.YSpan / 2;
			
			for (int i = 0; i < ro.XSpan; i += mapCellSize)
				for (int j = 0; j < ro.YSpan; j += mapCellSize)
					if (ro.isInObstacle(xOffset + i, yOffset + j))
						PathPlanner.updateCell((xOffset + i) / mapCellSize,
								(yOffset + j) / mapCellSize, -1);

		} else if (eo instanceof CircularObstacle) {
			CircularObstacle co = (CircularObstacle) eo;
			// expand the Obstacle by the robot size
			co = new CircularObstacle(co.Location, co.Radius + this.Radius,
					co.Velocity);
			// update blocked cells in the Circle
			int xOffset = co.Location.x - co.Radius;
			int yOffset = co.Location.y - co.Radius;
			for (int i = 0; i < 2 * co.Radius; i += mapCellSize)
				for (int j = 0; j < 2 * co.Radius; j += mapCellSize)
					if (co.isInObstacle(xOffset + i, yOffset + j))
						PathPlanner.updateCell((xOffset + i) / mapCellSize,
								(yOffset + j) / mapCellSize, -1);
		} else {// Point Obstacle
			PathPlanner.updateCell(eo.Location.x / mapCellSize, eo.Location.y
					/ mapCellSize, -1);
		}
	}

	@Override
	public void draw(Graphics2D g2d) {
		Graphics2D g = (Graphics2D) g2d.create();

		// draw the robot sensing range
		AlphaComposite ac = AlphaComposite.getInstance(AlphaComposite.SRC_OVER,
				0.2f);
		g.setComposite(ac);
		g.setColor(this.Color);
		if (DisplaySensingRange) {
			g.fillOval(this.Location.x - this.SensingRange, this.Location.y
					- this.SensingRange, 2 * this.SensingRange,
					2 * this.SensingRange);
		}

		// draw the oldPaths
		ac = AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.12f);
		g.setComposite(ac);
		g.setColor(this.Color.darker());
		int mapCellSize = Map.getCellSize();
		for (List<State> path : previousPaths)
			for (State s : path)
				g.fillRect(s.x * mapCellSize, s.y * mapCellSize,
						mapCellSize, mapCellSize);

		ac = AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.65f);
		g.setComposite(ac);

		g.setColor(this.Color.brighter());
		// draw the new Path
		for (State s : Path)
			g.fillRect(s.x * mapCellSize, s.y * mapCellSize, mapCellSize,
					mapCellSize);

		ac = AlphaComposite.getInstance(AlphaComposite.SRC);
		g.setComposite(ac);
		// draw the robot
		g.setColor(this.Color);
		g.fillOval(this.Location.x - this.Radius,
				this.Location.y - this.Radius, 2 * this.Radius, 2 * this.Radius);

		// draw its label
		FontMetrics fm = g.getFontMetrics();
		Rectangle2D rect = fm.getStringBounds(this.Label, g);
		g.setColor(java.awt.Color.white);
		
		g.drawString(this.Label, this.Location.x - (float) rect.getWidth() / 2,
				this.Location.y - (float) rect.getHeight() / 2 + fm.getAscent());

	}

	@Override
	public boolean isInObstacle(int x, int y) {// Robot can be a Circular
												// Obstacle
		return Math.pow(x - this.Location.x, 2)
				+ Math.pow(y - this.Location.y, 2) <= Math.pow(Radius, 2);
	}

	public boolean canCollide(int thisrx, int thisry, int thatx, int thaty,
			int leeway) {
		// returns if thatx,thaty is within collision region of this robot at
		// rx, ry location
		// returns if the point lies in the circle centered at thisrx,thisry
		return Math.pow(thatx - thisrx, 2) + Math.pow(thaty - thisry, 2) <= Math
				.pow(Radius + leeway, 2);
	}

	public int pathIndex = 0;
	public State currentState = null;

	public void step() {
		// move through the planned path
		if (RemainingPath.size() <= 0 || pathIndex + 1 >= RemainingPath.size())// nothing
																				// to
																				// do
			return;
		pathIndex++;
		currentState = RemainingPath.get(pathIndex);
		stepTo(currentState);
	}

	public void stepTo(int remainingPathIndex) {
		// move through the planned path
		if (RemainingPath.size() <= 0
				|| remainingPathIndex >= RemainingPath.size())// nothing to do
			return;
		this.pathIndex = remainingPathIndex;
		currentState = RemainingPath.get(remainingPathIndex);
		stepTo(currentState);
	}

	public List<State> getRemainingPath() {
		int startIndex = this.pathIndex >= this.RemainingPath.size() ? this.RemainingPath
				.size() - 1 : this.pathIndex;
		int endIndex = this.RemainingPath.size();
		return this.RemainingPath.subList(startIndex, endIndex);
	}

	public void stepTo(State s) {
		currentState = s;
		// step to the specified pathState
		int mapCellSize = Map.getCellSize();
		this.Location.x = s.x * mapCellSize + mapCellSize / 2;
		this.Location.y = s.y * mapCellSize + mapCellSize / 2;
	}
}