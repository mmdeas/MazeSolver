import java.lang.Thread;

import lejos.nxt.Button;
import lejos.nxt.LCD;
import lejos.nxt.LightSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;

public class MazeSolver
{
	private static final Maze maze = new Maze(7, 7);

	// Sensors
	private static final LightSensor leftLight = new LightSensor(SensorPort.S2);
	private static final LightSensor rightLight = new LightSensor(SensorPort.S3);
	private static final UltrasonicSensor sonar = new UltrasonicSensor(SensorPort.S1);

	// Robot Constants
	private static final int TRACK_SIZE = 56; //mm
	private static final int ROBOT_WIDTH = 174; //mm
	private static final int SONAR_TRUSTED_DISTANCE = 30; //cm
	private static final int AXLE_TO_LIGHTS = 68; //mm
	private static final int AXLE_TO_SONAR = 80; //mm

	// Motor
	private static DifferentialPilot dp = new DifferentialPilot(
												TRACK_SIZE,
												TRACK_SIZE,
												ROBOT_WIDTH,
												Motor.C, //Left
												Motor.B, //Right
												false
												);

	// Behaviour Constants
	private static final double STEER_SCALE = 300.0 / 20.0;
	private static final double SPEED_SCALE = dp.getMaxTravelSpeed() / 3;

	public static void main(String[] args)
	{
		// Behaviors ordered least to greatest priority
		Behavior[] bees = {followLine, backtrack};
		Arbitrator arbie = new Arbitrator(bees);
		LCD.drawString("Place me at A.", 0, 0);
		Button.waitForAnyPress();
		LCD.clearDisplay();
		arbie.start();
	}

	private static Behavior mapJunction = new Behavior()
	{
		boolean suppressed;

		@Override
		public boolean takeControl()
		{
			int right = rightLight.readValue();
			int left = leftLight.readValue();
			if (Math.abs(right - left) < 5
				&& right < 35
				&& left < 35)
			{
				return true;
			}
			return false;
		}

		@Override
		public void action()
		{
			dp.travel(AXLE_TO_LIGHTS);
			dp.rotate(360, true);
			boolean leftWasWhite = true;
			boolean rightWasWhite = true;
			while (!suppressed && dp.isMoving())
			{
				//TODO: can do this without storing all values
				int l = lightLeft.readValue();
				int r = lightRight.readValue();
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
							|| (r < 180 && l > 180 && Math.abs(l - r - 360) < 45))
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

			Collections.sort(angles);
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

			// TODO: choose direction, update maze object
		}
	}

	private static Behavior backtrack = new Behavior()
	{
		boolean suppressed;

		@Override
		public boolean takeControl()
		{
			if (sonar.getDistance < 2)
				return true;
			return false;
		}

		@Override
		public void action()
		{
			suppressed = false;
			maze.backtrack();
			while (!suppressed)
			{
				int right = rightLight.readValue();
				int left = leftLight.readValue();
				int diff = right - left;
				dp.steerBackward(diff * STEER_SCALE);
				dp.setTravelSpeed(SPEED_SCALE/diff);
				if (Math.abs(right - left) < 5 && right < 35 && left < 35)
					return;
			}
		}

		public void suppress() { suppressed = true; }
	}

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
				int diff = lightRight.readValue() - leftLight.readValue();
				dp.steer(diff * STEER_SCALE);
				dp.setTravelSpeed(SPEED_SCALE/diff)
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
	}

	// TODO: add maze changing functionality
	private class Maze
	{
		private boolean flashOn;
		private int[][] maze;
		private int width;
		private int height;
		private Thread flashThread;

		private int robotX = 0;
		private int robotY = 0;
		private int heading = 0; // 0 = N; 1 = W; 2 = S; 3 = E

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

			flashOn = true;
			flashThread = new Thread(flasher);
			flashThread.setPriority(Thread.MIN_PRIORITY);
			flashThread.start();
			for (int i = 0; i < width*2; i+=2)
			{
				for (int j = 0; j < height; j+=2)
				{
					LCD.setPixel(i, j, 1);
				}	
			}
		}

		// maze status
		public static void setEdge(int x, int y, boolean blocked)
		{
			maze[x][y] = blocked ? 0 : 1;
			LCD.setPixel((y%2)? x : x+1, y, blocked ? 0 : 1);
		}

		public static void setRelativeEdge(float a, int distance, boolean blocked)
		{
			int newHeading;
			if (45 < a && a < 135)
				newHeading = 1;
			else if (135 < a && 180 + 45)
				newHeading = 2;
			else if (270 - 45 < a && a < 270 + 45)
				newHeading = 3;
			else
				newHeading = 0;

			// TODO: logic here for changing robot to edge coords
		}

		// robot position status
		public static forward()
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

		public static backward()
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
								if (!(j % 2))
									x++;
								LCD.setPixel(x, j, flashOn ? 1 : 0);
							}
						}
					}
					flashOn = !flashOn;
					try { Thread.sleep(500); } catch (InterruptedException e) { break; }
				}
			}
		}
	}
}
