/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package astarplanner;

public class Node {
	public double gScore = 0;
	public double hScore = 0;
	public double fScore = 0;
	public NodeInfo NodeInfo;
	public Node cameFrom;

	@Override
	public boolean equals(Object oThat) {
		if (oThat instanceof Node)
			return ((Node) oThat).NodeInfo.compareTo(this.NodeInfo) == 0;
		else
			return false;
	}

	@Override
	public int hashCode() {
		return this.NodeInfo.hashCode();
	}
}
