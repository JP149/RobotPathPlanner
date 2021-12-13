/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package A_Star_Planner;

public class Node {
	public double gScore = 0;
	public double hScore = 0;
	public double fScore = 0;
	public NodeInfo NodeInfo;
	public Node cameFrom;

	// @Override
	// public int compareTo(Object oThat) {
	// if (oThat instanceof Node) {
	// return ((Node) oThat).NodeInfo.compareTo(this.NodeInfo);
	// } else
	// return -1;
	//
	// // if (oThat instanceof Node) {
	// // double diff = this.fScore - ((Node) oThat).fScore;
	// // if (diff == 0)
	// // return 0;
	// // else if (diff > 0)
	// // return 1;
	// // else
	// // return -1;
	// // } else
	// // return -1;
	// }

	@Override
	public boolean equals(Object oThat) {
		if (oThat instanceof Node)
			return ((Node) oThat).NodeInfo.compareTo(this.NodeInfo) == 0;
		else
			return false;
		// return this.compareTo(oThat) == 0;
	}

	@Override
	public int hashCode() {
		return this.NodeInfo.hashCode();
	}
}
