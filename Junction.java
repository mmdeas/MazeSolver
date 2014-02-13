import java.util.ArrayList;
import java.util.HashSet;
import java.util.Queue;
import java.util.Stack;

public class Junction
{
	public static enum Status {BLOCKED, FREE, UNKNOWN};
	public final int x;
	public final int y;
	private Status[] edges = {Status.UNKNOWN, Status.UNKNOWN, Status.UNKNOWN, Status.UNKNOWN};
	private Junction[] junctions = {null, null, null, null};
	private static Junction[][] maze = new Junction[7][7];

	public static Junction[][] createMaze()
	{
		for (int x = 0; x < 7; x++)
		{
			for (int y = 0; y < 7; y++)
			{
				maze[x][y] = new Junction(x, y);
			}
		}
		for (int x = 0; x < 7; x++)
		{
			for (int y = 0; y < 7; y++)
			{
				if (y != 0)
					maze[x][y].setJunction(2, maze[x][y-1]);
				if (y != 6)
					maze[x][y].setJunction(0, maze[x][y+1]);
				if (x != 0)
					maze[x][y].setJunction(1, maze[x-1][y]);
				if (x != 6)
					maze[x][y].setJunction(3, maze[x+1][y]);
			}
		}
		return maze;
	}

	private Junction(int x, int y)
	{
		this.x = x;
		this.y = y;
		if (y == 0)
			setEdge(2, Status.BLOCKED);
		if (y == 6)
			setEdge(0, Status.BLOCKED);
		if (x == 0)
			setEdge(1, Status.BLOCKED);
		if (x ==  6)
			setEdge(3, Status.BLOCKED);
	}

	public void setEdge(int heading, Status status)
	{
		edges[heading] = status;
	}

	public Status getEdge(int heading)
	{
		return edges[heading];
	}

	public void setJunction(int heading, Junction junction)
	{
		junctions[heading] = junction;
	}

	public Junction getJunction(int heading)
	{
		return junctions[heading];
	}
	
	@SuppressWarnings("deprecation")
	public Stack<Integer> pathToNearestUnknown()
	{
		Queue<BFSNode> leaves = new Queue<BFSNode>();
		HashSet<Junction> visited = new HashSet<Junction>();
		leaves.push(new BFSNode(null, this, -1));
		visited.add(this);
		while (!leaves.isEmpty())
		{
			BFSNode n = (BFSNode) leaves.pop();
			if (n.expand())
			{
				return n.pathFromRoot();
			}
			for (BFSNode child : n.children)
			{
				if (!visited.contains(child.junction))
				{
					leaves.push(child);
				}
			}
		}
		return null;
	}
	
	//TODO: A*
	/*
	public Stack<Integer> pathTo(int x, int y)
	{
		// No priority queue. :(
		ArrayList<AstarNode> leaves = new ArrayList<AstarNode>();
		
	}
	*/
	private class BFSNode
	{
		public final BFSNode parent;
		public final ArrayList<BFSNode> children = new ArrayList<BFSNode>();
		public final Junction junction;
		public final int headingFromParent;

		public BFSNode(BFSNode parent, Junction junction, int headingFromParent)
		{
			this.parent = parent;
			this.junction = junction;
			this.headingFromParent = headingFromParent;
		}

		public boolean expand()
		{
			for (int i = 0; i < 4; i++)
			{
				Status e = junction.getEdge(i);
				if (e == Status.UNKNOWN)
					return true;
				if (e == Status.FREE)
				{
					children.add(new BFSNode(this, junction.getJunction(i), i));
				}
			}
			return false;
		}

		public Stack<Integer> pathFromRoot()
		{
			Stack<Integer> path = new Stack<Integer>();
			BFSNode node = this;
			while (node.parent != null)
			{
				path.push(node.headingFromParent);
				node = node.parent;
			}
			return path;
		}
	}
	
	private class AstarNode extends BFSNode implements Comparable<AstarNode>
	{
		public final int g;
		public final int f;
		public AstarNode(AstarNode parent, Junction junction, int headingFromParent, int g, Junction goal)
		{
			super(parent, junction, headingFromParent);
			this.g = g;
			this.f = g + Math.abs(goal.x - junction.x) + Math.abs(goal.y - junction.y);
		}
		
		@Override
		public int compareTo(AstarNode n)
		{
			return new Integer(f).compareTo(new Integer(n.f));
		}
		
	}
}