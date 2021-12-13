/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
**/
package A_Star_Planner;

@SuppressWarnings({ "serial", "rawtypes" })
public class NodeInfo implements Comparable, java.io.Serializable {
	public int[] Location;

	public static boolean checkEqual(int[] location1, int[] location2) {
		if (location1 == null || location2 == null)
			return false;

		if (location1.length == location2.length)
			for (int i = 0; i < location2.length; i++)
				if (location2[i] != location1[i])
					return false;
		return true;
	}

	@Override
	public int compareTo(Object oThat) {

		return this.equals(oThat) ? 0 : -1;
	}

	@Override
	public boolean equals(Object oThat) {
		if (this == oThat)
			return true;
		if (!(oThat instanceof NodeInfo))
			return false;

		NodeInfo niThat = (NodeInfo) oThat;

		// Nodes in the same Location?
		if (niThat.Location.length == this.Location.length)
			for (int i = 0; i < this.Location.length; i++)
				if (this.Location[i] != niThat.Location[i])
					return false;
		return true;
	}

	@Override
	public int hashCode() {
		int h = 0;
		for (int i = 0; i < this.Location.length; i++)
			h += hash(this.Location[i]);
		return h;
	}

	int hash(int x) {
		x = ((x >> 16) ^ x) * 0x45d9f3b;
		x = ((x >> 16) ^ x) * 0x45d9f3b;
		x = ((x >> 16) ^ x);
		return x;
	}
}
