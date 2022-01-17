package simulator;

import java.util.ArrayList;
import java.util.List;

import dstarlite.DStarLite;
import dstarlite.State;
import simulator.env.obstacles.CircularObstacle;
import simulator.env.obstacles.EnvironmentObject;
import simulator.env.obstacles.RectangularObstacle;

public class RobotPlanner {
	public List<State> Path = new ArrayList<State>();
	public List<State> RemainingPath;
	public ArrayList<List<State>> previousPaths = new ArrayList<List<State>>();;
	public ArrayList<EnvironmentObject> knownObstacles = new ArrayList<EnvironmentObject>();
	public DStarLite DStarPathPlanner;
	public Map Map;
	public boolean DisplaySensingRange;
	public boolean hasPlannedOnce;
	public int pathIndex;
	public State currentState = new State();
	private Robot robot;

	public RobotPlanner(boolean isReplanning, boolean displaySensingRange, boolean hasPlannedOnce, int pathIndex,
			State currentState, Robot robot) {
		DisplaySensingRange = displaySensingRange;
		this.hasPlannedOnce = hasPlannedOnce;
		this.pathIndex = pathIndex;
		this.currentState = currentState;
		this.robot = robot;
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
		
		long elapsed = System.currentTimeMillis();
		DStarPathPlanner.updateStart(currentState.x, currentState.y);
		DStarPathPlanner.replan();
		if (this.Path.size() > 1)
			previousPaths.add(this.Path);
		this.Path = DStarPathPlanner.getPath();
		this.RemainingPath = this.Path;
		pathIndex = 0;

		elapsed = System.currentTimeMillis() - elapsed;
		System.out.println("D*Lite Replan: " + elapsed + "ms");
	}

	public boolean processEnvironment(ArrayList<EnvironmentObject> environmentObjects) {
		// detect environment changes
		// process newly detected obstacles
		// check for new objects within the robot sensing range
		boolean replanNecessary = false;
		for (EnvironmentObject eo : environmentObjects) {

			boolean inSensingRange = eo.isInSensingRange(robot.Location, robot.SensingRange);

			if (!knownObstacles.contains(eo) && inSensingRange)
				replanNecessary = processNewKnownObject(eo);
		}
		return replanNecessary;
	}

	public boolean processNewKnownObject(EnvironmentObject eo) {
		boolean replanRequired = false;
		
		if (knownObstacles.contains(eo))
			return replanRequired;
		
		knownObstacles.add(eo);// obstacle is now known to the robot -
								// assumes obstacles are stationary
		eo.registerRobot(this.robot.Color);

		updateCells(eo);// update the corresponding
						// obstacle cells
		replanRequired = true;// re-plan necessary

		return replanRequired;// re-plan unnecessary - object is already known or not in
		// sensing range
	}

	private void updateCells(EnvironmentObject eo) {
		int mapCellSize = Map.getCellSize();
		
		eo.updateCells(DStarPathPlanner, mapCellSize, robot);
	}

	public boolean canCollide(int thisrx, int thisry, int thatx, int thaty, int leeway) {
		// returns if thatx,thaty is within collision region of this robot at
		// rx, ry location
		// returns if the point lies in the circle centered at thisrx,thisry
		return Math.pow(thatx - thisrx, 2) + Math.pow(thaty - thisry, 2) <= Math.pow(robot.Radius + leeway, 2);
	}

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
		if (RemainingPath.size() <= 0 || remainingPathIndex >= RemainingPath.size())// nothing to do
			return;
		this.pathIndex = remainingPathIndex;
		currentState = RemainingPath.get(remainingPathIndex);
		stepTo(currentState);
	}

	public List<State> getRemainingPath() {
		int startIndex = this.pathIndex;
		
		if(startIndex >= this.RemainingPath.size())
			startIndex = this.RemainingPath.size()-1;
		
		int endIndex = this.RemainingPath.size();
		return this.RemainingPath.subList(startIndex, endIndex);
	}

	public void stepTo(State s) {
		currentState = s;
		// step to the specified pathState
		int mapCellSize = Map.getCellSize();
		this.robot.Location.x = s.x * mapCellSize + mapCellSize / 2;
		this.robot.Location.y = s.y * mapCellSize + mapCellSize / 2;
	}
}