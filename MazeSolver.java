import java.lang.Thread;
import java.util.ArrayList;
import java.util.EmptyStackException;
import java.util.HashSet;
import java.util.Queue;
import java.util.Stack;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class MazeSolver
{
	private static Maze maze;

	// Sensors
	private static final LightSensor leftLight = new LightSensor(SensorPort.S2);
	private static final LightSensor rightLight = new LightSensor(SensorPort.S3);
	private static final UltrasonicSensor sonar = new UltrasonicSensor(SensorPort.S1);

	// Robot Constants
	private static final int TRACK_SIZE = 56; //mm
	private static final int ROBOT_WIDTH = 170; //mm
	private static final int SONAR_TRUSTED_DISTANCE = 30; //cm
	private static final int AXLE_TO_LIGHTS = 65; //mm
	private static final int AXLE_TO_SONAR = 80; //mm

	// Motor
	private static DifferentialPilot dp = new DifferentialPilot(
												TRACK_SIZE,
												TRACK_SIZE,
												ROBOT_WIDTH,
												Motor.C, //Left
												Motor.A, //Right
												false
												);

	// Behaviour Constants
	private static final double STEER_SCALE = 300.0 / 20.0;
	private static final double SPEED_SCALE = dp.getMaxTravelSpeed() / 3;
	private static final double SPEED = dp.getMaxTravelSpeed() / 8;
	
	private static int right;
	private static int left;

	public static void main(String[] args)
	{
		// Behaviors ordered least to greatest priority
		Behavior[] bees = {followLine, mapJunction};//, backtrack};
		Arbitrator arbieTheExplorer = new Arbitrator(bees);
		LCD.drawString("Place me at A.", 0, 0);
		Button.waitForAnyPress();
		LCD.clearDisplay();
		right = rightLight.readValue();
		left = leftLight.readValue();
		maze = new Maze(7, 7);
		arbieTheExplorer.start();
	}

	private static Behavior mapJunction = new Behavior()
	{
		boolean suppressed;

		@Override
		public boolean takeControl()
		{
			if (Math.abs(right - left) < 5
				&& right < 40
				&& left < 40)
			{
				return true;
			}
			return false;
		}

		@Override
		public void action()
		{
			suppressed = false;
			dp.travel(AXLE_TO_LIGHTS);
			dp.rotate(360, true);
			boolean leftWasWhite = true;
			boolean rightWasWhite = true;
			ArrayList<Float> leftAngles = new ArrayList<Float>();
			ArrayList<Float> rightAngles = new ArrayList<Float>();
			while (!suppressed && dp.isMoving())
			{
				//TODO: can do this without storing all values
				int l = leftLight.readValue();
				int r = rightLight.readValue();
				if (l < 40 && leftWasWhite)
				{
					leftAngles.add(dp.getAngleIncrement());
					leftWasWhite = false;
				}
				else if (l > 45)
				{
					leftWasWhite = true;
				}
				
				if (r < 40 && rightWasWhite)
				{
					rightAngles.add(dp.getAngleIncrement());
					rightWasWhite = false;
				}
				else if (r > 45)
				{
					rightWasWhite = true;
				}
			}
			if (suppressed)
				dp.stop();

			ArrayList<Float> angles = new ArrayList<Float>();
			for (Float l : leftAngles)
			{
				for (Float r : rightAngles)
				{
					if (Math.abs(l - r) < 45 
							|| (r < 180 
								&& l > 180 
								&& Math.abs(l - r - 360) < 45
								)
						)
					{
						if (r < 180 && l > 180)
							l -= 360;
						float a = (l + r) / 2;
						if (a > 360 - 45)
							a -= 360;
						angles.add((l + r) / 2);
						break;
					}
				}
			}

			for (int i = 0; i < angles.size(); i++)
			{
				for (int j = 1; j < angles.size() - i; j++)
				{
					if (angles.get(j-1) > angles.get(j))
					{
						float tmp = angles.get(j-1);
						angles.set(j-1, angles.get(j));
						angles.set(j, tmp);
					}
				}
			}
			// angles should now be forward, left, back, right

			float previous = 0.0f;
			int[] echoes = new int[8];
			for (int i = 0; i < angles.size(); i++)
			{
				int j;
				float a = angles.get(i);
				dp.rotate(angles.get(i) - previous);

				sonar.ping();
				int n = sonar.getDistances(echoes);
				int min = 255;
				for (j = 0; j < n; j++)
				{
					min = echoes[j] < min ? echoes[j] : min;	
				}

				min += AXLE_TO_SONAR/10;

				if (min > SONAR_TRUSTED_DISTANCE)
				{
					for (j = 1; j * 30 < SONAR_TRUSTED_DISTANCE; j++)
						maze.setRelativeEdge(a, j, false);
				}
				else
				{
					for (j = 1; j * 30 < min; j++)
						maze.setRelativeEdge(a, j, false);
					maze.setRelativeEdge(a, j, true);
				}

				previous = a;
			}

			int h;
			try
			{
				h = maze.getNextRelativeHeading();
			}
			catch (EmptyStackException e)
			{
				maze.pathToClosestUnexplored();
				h = maze.getNextRelativeHeading();
			}
			maze.setRelativeHeading(h);
			maze.forward();
			float a = angles.get(h);
			dp.rotate(a - previous);
			left = leftLight.readValue();
			right = rightLight.readValue();
		}

		@Override
		public void suppress() { suppressed = true; }
	};

	private static Behavior backtrack = new Behavior()
	{
		boolean suppressed;

		@Override
		public boolean takeControl()
		{
			if (sonar.getDistance() < 2)
				return true;
			return false;
		}

		@Override
		public void action()
		{
			suppressed = false;
			Thread rt = new Thread(reverseSound);
			rt.start();
			maze.backward();
			while (!suppressed)
			{
				int right = rightLight.readValue();
				int left = leftLight.readValue();
				int diff = right - left;
				dp.steerBackward(diff * STEER_SCALE);
				dp.setTravelSpeed(SPEED_SCALE/diff);
				if (Math.abs(right - left) < 5 && right < 35 && left < 35)
					break;
			}
			rt.interrupt();
		}

		public void suppress() { suppressed = true; }
	};

	private static Behavior followLine = new Behavior()
	{
		boolean suppressed;

		// Default behaviour: always try to take control
		@Override
		public boolean takeControl() { return true; }

		@Override
		public void action()
		{
			suppressed = false;
			while (!suppressed)
			{
				right = rightLight.readValue();
				left = leftLight.readValue();
				int diff = right - left;
				dp.steer(diff * STEER_SCALE);
				dp.setTravelSpeed(SPEED);
			}
		}

		@Override
		public void suppress() { suppressed = true; }
	};

	private static Runnable reverseSound = new Runnable()
	{
		@Override
		public void run()
		{
			while (!Thread.currentThread().isInterrupted())
			{
				Sound.playTone(1000, 500);
				try
				{
					Thread.sleep(1000);
				}
				catch (InterruptedException e)
				{
					break;
				}
			}
		}
	};

	private static class Maze
	{
		private boolean flashOn;
		private int[][] maze;
		private int width;
		private int height;
		private Thread flashThread;

		private int robotX = 0;
		private int robotY = 0;
		private int heading = 0; // 0 = N; 1 = W; 2 = S; 3 = E

		private Stack<Integer> path = new Stack<Integer>();

		public Maze(int width, int height)
		{
			this.width = width;
			this.height = height;

			maze = new int[width][height];
			for (int j = 0; j < height; j++)
			{
				for (int i = 0; i < width-1; i++)
				{
					maze[i][j] = -1;
				}
				maze[width-1][j] = j == height-1 ? 1 : 0;
			}

			/*
			flashOn = true;
			flashThread = new Thread(flasher);
			flashThread.setPriority(Thread.MIN_PRIORITY);
			flashThread.start();
			*/
			for (int i = 0; i < width*2; i+=2)
			{
				for (int j = 0; j < height; j+=2)
				{
					LCD.setPixel(i, j, 1);
				}	
			}
		}

		// navigation
		public int getNextRelativeHeading() throws EmptyStackException
		{
			int globalHeading = path.pop();
			return globalHeading - heading;
		}

		// bfs
		public boolean pathToClosestUnexplored()
		{
			Queue<BfsNode> leaves = new Queue<BfsNode>();
			HashSet<Integer> visited = new HashSet<Integer>();
			BfsNode root = new BfsNode(null, robotX, robotY, -1);
			leaves.push(root);
			visited.add(0);
			while (!leaves.empty())
			{
				BfsNode n = (BfsNode) leaves.pop();
				int h = n.expand();
				if (h != -1)
				{
					path = n.pathFromRoot(h);
					return true;
				}
				for (BfsNode child : n.children)
				{
					if (!visited.contains((child.junctionX << 8 + child.junctionY)))
					{
						visited.add((child.junctionX << 8 + child.junctionY));
						leaves.push(child);
					}
				}
			}
			return false;
		}

		private class BfsNode
		{
			public final BfsNode parent;
			public final ArrayList<BfsNode> children = new ArrayList<BfsNode>();
			public final int junctionX;
			public final int junctionY;
			public final int headingFromParent;

			public BfsNode(BfsNode parent, int junctionX, int junctionY, int headingFromParent)
			{
				this.parent = parent;
				this.junctionX = junctionX;
				this.junctionY = junctionY;
				this.headingFromParent = headingFromParent;
			}

			public int expand()
			{
				for (int i = 0; i < 4; i++)
				{
					int e = getEdge(junctionX, junctionY, i);
					if (e == -1)
						return i;
					if (e == 1)
					{
						int newy = junctionY;
						int newx = junctionX;
						switch (i)
						{
						case 0:
							newy++;
							break;
						case 1:
							newx--;
							break;
						case 2:
							newy--;
							break;
						case 3:
							newx++;
							break;
						}
						children.add(new BfsNode(this, newx, newy, i));
					}
				}
				return -1;
			}

			public Stack<Integer> pathFromRoot(int lastHeading)
			{
				Stack<Integer> path = new Stack<Integer>();
				path.push(lastHeading);
				BfsNode node = this;
				while (node.parent != null)
				{
					path.push(node.headingFromParent);
					node = node.parent;
				}
				return path;
			}
		}

		// maze status
		public void setEdge(int x, int y, boolean blocked)
		{
			maze[x][y] = blocked ? 0 : 1;
			LCD.setPixel((y%2)==1 ? x : x+1, y, blocked ? 0 : 1);
		}

		public void setRelativeEdge(float a, int distance, boolean blocked)
		{
			int edgeX, edgeY;
			edgeX = edgeY = 0;
			switch (globalHeadingFromAngle(a))
			{
				case 0:
					edgeX = robotX;
					edgeY = 2 * robotY;
					break;
				case 1:
					edgeX = robotX - 1;
					edgeY = 2 * robotY;
					break;
				case 2:
					edgeX = robotX;
					edgeY = 2 * robotY - 1;
					break;
				case 3:
					edgeX = robotX + 1;
					edgeY = 2 * robotY;
					break;
				default:
					Sound.beep();
					Sound.buzz();
			}

			setEdge(edgeX, edgeY, blocked);
		}

		private int getEdge(int x, int y, int heading)
		{
			try
			{
				switch (heading)
				{
				case 0:
					return maze[x][2*y];
				case 1:
					return maze[x-1][2*y];
				case 2:
					return maze[x][2*y-1];
				case 3:
					return maze[x+1][2*y];
				default:
					Sound.beep();
					Sound.buzz();
					return -255;
				}
			}
			catch (Exception e)
			{
				return 0;
			}
		}

		public void setRelativeHeading(float a)
		{
			heading = globalHeadingFromAngle(a);
		}

		private int globalHeadingFromAngle(float a)
		{
			if (45 < a && a < 135)
				return (heading + 1) % 4;
			else if (135 < a && a < 180 + 45)
				return (heading + 2) % 4;
			else if (270 - 45 < a && a < 270 + 45)
				return (heading + 3) % 4;
			else
				return heading;
		}

		// robot position status
		public void forward()
		{
			switch (heading)
			{
				case 0:
					robotY++;
					break;
				case 1:
					robotX--;
					break;
				case 2:
					robotY--;
					break;
				case 3:
					robotX++;
					break;
			}
		}

		public void backward()
		{
			switch (heading)
			{
				case 0:
					robotY--;
					break;
				case 1:
					robotX++;
					break;
				case 2:
					robotY++;
					break;
				case 3:
					robotX--;
					break;
			}
		}

		// internal
		private Runnable flasher = new Runnable()
		{
			public void run()
			{
				while (!Thread.currentThread().isInterrupted())
				{
					for (int i = 0; i < width; i++)
					{
						for (int j = 0; j < height; j++)
						{
							if (maze[i][j] == -1)
							{
								int x = 2*i;
								if ((j % 2) == 0)
									x++;
								LCD.setPixel(x, j, flashOn ? 1 : 0);
							}
						}
					}
					flashOn = !flashOn;
					try { Thread.sleep(500); } catch (InterruptedException e) { break; }
				}
			}
		};
	}
}
