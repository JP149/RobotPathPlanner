/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package A_Star_Planner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;

import simulator.Robot;
import DStarLite.State;

public class VelocityPlanner_A_Star {
	public int Dimensions;
	public int[] DimensionSizes;
	List<Robot> Robots;
	List<List<State>> Paths;
	public double D_CELL_TO_CELL;
	public int Leeway = 10;
	public Node Start;
	public Node Goal;

	public VelocityPlanner_A_Star(List<Robot> Robots) {
		this.Robots = Robots;
		this.Paths = new ArrayList<List<State>>();
	}

	private void init() {
		this.Paths = new ArrayList<List<State>>();

		for (int i = 0; i < Robots.size(); i++) {
			Robot r = Robots.get(i);
			List<State> subPath = r.getRemainingPath();
			r.RemainingPath = subPath;
			this.Paths.add(subPath);
			if (r.Radius > Leeway)
				Leeway = r.Radius;
		}
		this.Dimensions = Paths.size();
		this.DimensionSizes = new int[this.Dimensions];
		for (int i = 0; i < this.Dimensions; i++) {
			List<State> path = Paths.get(i);
			this.DimensionSizes[i] = path.size();
		}
		D_CELL_TO_CELL = Math.sqrt(Dimensions);

		// start node is [0,0,...,0] - start state of the path of all robots
		this.Start = new Node();
		this.Start.NodeInfo = new NodeInfo();
		this.Start.NodeInfo.Location = new int[this.Dimensions];// new integers
																// are
																// initialized
																// to zero

		// goal node is [size(dimension 1)-1,size(dimension
		// 2)-1,...size(dimension
		// N)-1] - the accomplished end state of the
		// path of all robots
		this.Goal = new Node();
		this.Goal.NodeInfo = new NodeInfo();
		this.Goal.NodeInfo.Location = this.DimensionSizes.clone();
		for (int i = 0; i < this.Goal.NodeInfo.Location.length; i++) {
			this.Goal.NodeInfo.Location[i] -= 1;
		}
	}

	public int getIndex(Robot r) {
		return Robots.indexOf(r);
	}

	public List<Node> replan() {
		init();
		List<Node> path = AstarSearch(this.Start, this.Goal);
		return path;
	}

	public double GCalc(Node node, Node parent) {
		if (node == null || parent == null)
			return 0;
		else
			return parent.gScore + eDistance(node, parent);
	}

	public double eDistance(Node a, Node b) {
		// double modifier = prioritize ? .5 : 1;
		double d = 0;
		for (int i = 0; i < Dimensions; i++) {
			// double modifier = prioritize ? (i + 1.0) / Dimensions : 1;
			// d += modifier
			// * Math.abs(a.NodeInfo.Location[i] - b.NodeInfo.Location[i]);
			d += Math.abs(a.NodeInfo.Location[i] - b.NodeInfo.Location[i]);
		}
		d = Math.sqrt(d);
		return d;
	}

	public double HCalc(Node node, Node goal) {

		if (node == null)
			return 1e+7;
		else {
			// N Dimensional Euclidean distance
			return eDistance(node, goal);
		}
	}

	public List<Node> AstarSearch(Node start, Node goal) {
		PriorityQueue<Node> openSet = new PriorityQueue<Node>(20,
				new Comparator<Node>() {//used for sorting by fscore
					@Override
					public int compare(Node n1, Node n2) {
						return n1.fScore == n2.fScore ? 0
								: n1.fScore > n2.fScore ? -1 : 1;
					}

				});
		List<Node> closedSet = new ArrayList<Node>();
		Node currNode = start;
		currNode.gScore = GCalc(start, start);
		currNode.hScore = HCalc(start, goal);
		currNode.fScore = currNode.gScore + currNode.hScore;
		openSet.add(start);
		while (!openSet.isEmpty()) {
			currNode = openSet.poll();
			if (currNode.NodeInfo.compareTo(goal.NodeInfo) == 0)// goal reached
				return ReconstructPath(currNode);
			closedSet.add(currNode);
			List<Node> neighbors = expandChildren(currNode);
			for (Node neighbor : neighbors) {
				int neighborIndex = closedSet.indexOf(neighbor);
				neighbor.gScore = neighborIndex == -1 ? Double.POSITIVE_INFINITY
						: closedSet.get(neighborIndex).gScore;
				double tentativeGScore = GCalc(neighbor, currNode);

				if (neighborIndex != -1)
					if (tentativeGScore >= neighbor.gScore)
						// indicates a better path than the previously
						// considered path to the neighbor
						continue;// will now re-add this possibility to the
									// openset
				boolean neighborInOpenSet = openSet.contains(neighbor);
				if (!neighborInOpenSet || tentativeGScore < neighbor.gScore) {
					neighbor.cameFrom = currNode;
					neighbor.gScore = tentativeGScore;
					neighbor.hScore = HCalc(neighbor, goal);
					neighbor.fScore = neighbor.gScore + neighbor.hScore;
					if (!neighborInOpenSet)
						openSet.add(neighbor);
					else {
						// re-add to update the fscore and force resorting of
						// the openset due to the new value
						openSet.remove(neighbor);
						openSet.add(neighbor);
					}
				}
			}
		}
		return null;// path not found
	}

	private List<Node> expandChildren(Node parent) {
		List<Node> children = new ArrayList<Node>();
		// Get N Dimensional Neighbors of the node
		// each dimension allows movement forward, backward or no movement
		// yielding 3^N possible children
		// the search can be reduced to monotonically increasing paths - only
		// forward or no movement
		// yeilding 2^N possible children
		// the parent,grandparent, and obstacles are to be ignored

		int dimFreedom = 2;
		// represents 2^N children each with location index per dimension N
		int numberOfChildren = (int) Math.pow(dimFreedom, Dimensions);
		int[][] childrenLocations = new int[numberOfChildren][Dimensions];

		// find all possible combinations of movement
		for (int d = 0; d < Dimensions; d++) {
			int x = parent.NodeInfo.Location[d];// parent's current position in
												// the dimension
			int divisor = (int) Math.pow(dimFreedom, d);
			for (int i = 0; i < numberOfChildren; i++) {
				switch ((i / divisor) % dimFreedom) {
				case 0:// still
					childrenLocations[i][d] = x;
					break;
				case 1:// forward movement
					childrenLocations[i][d] = x + 1;
					break;
				// case 2://backward movement
				// childrenLocations[i][d] = x -1;
				// break;
				}
			}
		}

		// filter out the parents,grandparents, obstacles, and out of bound
		// Locations
		for (int i = 0; i < childrenLocations.length; i++) {
			int[] childLocation = childrenLocations[i];
			boolean inDimensionBounds = true;
			for (int d = 0; d < childLocation.length; d++) {
				int locationInDim = childLocation[d];
				// check if the location is valid - within bounds of each
				// dimension
				if (locationInDim < 0
						|| locationInDim >= this.DimensionSizes[d]) {
					inDimensionBounds = false;
					break;
				}
			}
			if (!inDimensionBounds
					|| NodeInfo.checkEqual(childLocation,
							parent.NodeInfo.Location)
					|| (parent.cameFrom != null && NodeInfo.checkEqual(
							childLocation, parent.cameFrom.NodeInfo.Location))
					|| inObstacle(childLocation, this.Leeway))
				continue;// skip this neighbor
			else {
				// a valid traversible neighbor
				Node newNeighbor = new Node();
				newNeighbor.NodeInfo = new NodeInfo();
				newNeighbor.NodeInfo.Location = childLocation;
				children.add(newNeighbor);
			}
		}
		return children;
	}

	private List<Node> ReconstructPath(Node endNode) {
		List<Node> path = new ArrayList<Node>();
		Node currentNode = endNode;

		while (currentNode != null) {
			path.add(currentNode);
			currentNode = currentNode.cameFrom;
		}
		Collections.reverse(path);
		return path;
	}

	private boolean inObstacle(int[] location1, int leeway) {
		// Velocity Planning Collision Check
		// if any robot state in this location causes collision with another
		// robot, return false
		for (int r = 0; r < Robots.size(); r++) {
			Robot ri = Robots.get(r);
			List<State> ripath = Paths.get(r);
			State riState = ripath.get(location1[r]);
			int cellSize = ri.Map.CellSize;
			int rix = riState.x * cellSize;
			int riy = riState.y * cellSize;

			for (int rj = 0; rj < location1.length; rj++) {
				if (r == rj)
					continue;// state is this robot's state - ignore collision
								// check
				List<State> rjpath = Paths.get(rj);
				int stateIndex = location1[rj];
				State rjState = rjpath.get(stateIndex);// get robots
														// state
				if (ri.canCollide(rix, riy, rjState.x * cellSize, rjState.y
						* cellSize, leeway))
					return true;
			}
		}
		return false;
	}
}