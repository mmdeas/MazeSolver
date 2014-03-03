import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.NoSuchElementException;
import java.util.Queue;
import java.util.Stack;

import lejos.nxt.LCD;

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
		if (x == 6)
			setEdge(3, Status.BLOCKED);
	}

	public void setEdge(int heading, Status status)
	{
		if (edges[heading] == Status.UNKNOWN)
		{
			edges[heading] = status;
			if (junctions[heading] != null)
				junctions[heading].setEdge((heading+2) % 4, status);
		}
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
		for (int i = 0; i < 4; i++)
		{
			LCD.drawString(getEdge(i).toString(), 0, i+2);
		}
		Queue<BFSNode> leaves = new Queue<BFSNode>();
		HashSet<Junction> visited = new HashSet<Junction>();
		leaves.push(new BFSNode(null, this, -1));
		visited.add(this);
		while (!leaves.isEmpty())
		{
			BFSNode n = (BFSNode) leaves.pop();
			visited.add(n.junction);
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
	
	// A*
	public Stack<Integer> pathTo(int x, int y)
	{
		PriorityQueue<AstarNode> leaves = new PriorityQueue<AstarNode>();
		Junction goal = maze[x][y];
		HashSet<Junction> expanded = new HashSet<Junction>();
		AstarNode root = new AstarNode(null, this, -1, 0, goal);
		leaves.enqueue(root);
		while (!leaves.isEmpty())
		{
			AstarNode n = leaves.pop();
			if (expanded.contains(n.junction))
				continue;
			if (n.expand())
				return n.pathFromRoot();
			expanded.add(n.junction);
			for (AstarNode child : n.children)
			{
				if (!expanded.contains(child))
				{
					leaves.enqueue(child);
				}
			}
		}
		return null;
	}

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
		public final Junction goal;
		public final int g;
		public final int f;
		public final ArrayList<AstarNode> children = new ArrayList<AstarNode>();
		public AstarNode(AstarNode parent, Junction junction, int headingFromParent, int g, Junction goal)
		{
			super(parent, junction, headingFromParent);
			this.g = g;
			this.f = g + Math.abs(goal.x - junction.x) + Math.abs(goal.y - junction.y);
			this.goal = goal;
		}

		/** Returns true when this node being expanded is the goal. */
		@Override
		public boolean expand()
		{
			if (junction == goal)
				return true;
			for (int i = 0; i < 4; i++)
			{
				Status e = junction.getEdge(i);
				if (e == Status.FREE)
				{
					children.add(new AstarNode(this, junction.getJunction(i), i, g + 1, goal));
				}
			}
			return false;
		}
		
		@Override
		public int compareTo(AstarNode n)
		{
			return new Integer(f).compareTo(new Integer(n.f));
		}
		
	}

	private class PriorityQueue<E extends Comparable>
	{
		private final LinkedList<E> queue = new LinkedList<E>();

		public E pop() throws NoSuchElementException
		{
			return queue.remove(0);
		}

		public void enqueue(E e)
		{
			for (int i = 0; i < queue.size(); i++)
			{
				if (e.compareTo(queue.get(i)) < 0)
				{
					queue.add(i, e);
					return;
				}
			}
			queue.add(e);
		}

		public boolean isEmpty() { return queue.isEmpty(); }
	}
}