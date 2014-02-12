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
	// Sensors
	private static final LightSensor leftLight = new LightSensor(SensorPort.S2);
	private static final LightSensor rightLight = new LightSensor(SensorPort.S3);
	private static final UltrasonicSensor sonar = new UltrasonicSensor(SensorPort.S1);

	// Robot Constants
	private static final int TRACK_SIZE = 56; //mm
	private static final int ROBOT_WIDTH = 174; //mm
	private static final int SONAR_TRUSTED_DISTANCE = 30; //cm

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
		sonar.continuous();

		// Behaviors ordered least to greatest priority
		Behavior[] bees = {followLine, backtrack};
		Arbitrator arbie = new Arbitrator(bees);
		LCD.drawString("Place me at A.", 0, 0);
		Button.waitForAnyPress();
		LCD.clearDisplay();
		arbie.start();
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
			while (!suppressed)
			{
				int diff = lightRight.readValue() - leftLight.readValue();
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

	private class Maze
	{
		private boolean flashOn;
		private int[][] maze;
		private int width;
		private int height;
		private Thread flashThread;

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

		public static void setEdge(int x, int y, boolean blocked)
		{
			maze[x][y] = blocked ? 0 : 1;
			LCD.setPixel((y%2)? x : x+1, y, blocked ? 0 : 1);
		}

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
