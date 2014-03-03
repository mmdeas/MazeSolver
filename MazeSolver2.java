import java.util.ArrayList;
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

public class MazeSolver2
{
	private static Junction[][] maze = Junction.createMaze();
	private static Stack<Integer> path = new Stack<Integer>();
	
	// Sensors
	private static final LightSensor leftLight = new LightSensor(SensorPort.S2);
	private static final LightSensor rightLight = new LightSensor(SensorPort.S3);
	private static final UltrasonicSensor sonar = new UltrasonicSensor(SensorPort.S1);

	// Robot Constants
	private static final int TRACK_SIZE = 56; //mm
	private static final int ROBOT_WIDTH = 172; //mm
	private static final int SONAR_TRUSTED_DISTANCE = 30; //cm
	private static final int AXLE_TO_LIGHTS = 65; //mm
	private static final int AXLE_TO_SONAR = 80; //mm
	private static final int AWESOME_LEVEL = 83; //au

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
	private static final double SPEED = dp.getMaxTravelSpeed() / 2;
	
	private static int right;
	private static int left;
	private static boolean exploring = true;
	
	public static void main(String[] args)
	{
		// Behaviours ordered least to greatest priority
		Behavior[] bees = {followLine, mapJunction, followPath};
		Arbitrator arbie = new Arbitrator(bees);
		LCD.drawString("Place me at A.", 0, 0);
		Button.waitForAnyPress();
		LCD.clearDisplay();
		right = rightLight.readValue();
		left = leftLight.readValue();
		dp.setTravelSpeed(SPEED);
		dp.setRotateSpeed(dp.getMaxRotateSpeed()/2);
		arbie.start();
	}
	
	
	private static Behavior followPath = new Behavior()
	{
		boolean suppressed;
		
		@Override
		public boolean takeControl()
		{
			if (!exploring && !path.empty()
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
			try { Thread.sleep(250); } catch (InterruptedException e) {}
			while (!suppressed && dp.isMoving())
			{
				//TODO: can do this without storing all values
				int l = leftLight.readValue();
				int r = rightLight.readValue();
				if (l < 38 && leftWasWhite)
				{
					leftAngles.add(dp.getAngleIncrement());
					leftWasWhite = false;
				}
				else if (l > 42)
				{
					leftWasWhite = true;
				}
				
				if (r < 38 && rightWasWhite)
				{
					rightAngles.add(dp.getAngleIncrement());
					rightWasWhite = false;
				}
				else if (r > 42)
				{
					rightWasWhite = true;
				}
			}
			if (suppressed)
				dp.stop();
			
			LCD.drawInt(leftAngles.size(), 0, 3);
			LCD.drawInt(rightAngles.size(), 0, 4);

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
			
			if (path == null)
			{
				Sound.beep();
				LCD.drawString("Hello", 0, 0);
				Button.waitForAnyPress();
			}
			if (path.empty())
			{
				if (Robot.getJunction() == maze[0][0])
				{
					path = Robot.getJunction().pathTo(4, 4);
				}
				else
				{
					Sound.beep();
					dp.rotate(-1080);
					return;
				}
			}
			
			int heading = path.pop();
			LCD.drawInt(heading, 4, 0);
			LCD.drawInt(Robot.toLocal(heading), 4, 1);
			float angle = 1080;
			for (int i = 0; i < angles.size(); i++)
			{
				if (Math.abs(angles.get(i) - Robot.toLocal(heading)*90) < 45)
				{
					angle = angles.get(i);
					break;
				}
			}
			
			dp.rotate(angle);
			
			Robot.heading = heading;
			Robot.forward();
			
			left = leftLight.readValue();
			right = rightLight.readValue();
		}

		@Override
		public void suppress() { suppressed = true; }
	};
	
	private static Behavior mapJunction = new Behavior()
	{
		boolean suppressed;

		@Override
		public boolean takeControl()
		{
			if (exploring
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
			//!!!
			LCD.drawInt(Robot.x, 0, 0);
			LCD.drawInt(Robot.y, 0, 1);
			//!!!
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
				if (l < 38 && leftWasWhite)
				{
					leftAngles.add(dp.getAngleIncrement());
					leftWasWhite = false;
				}
				else if (l > 42)
				{
					leftWasWhite = true;
				}
				
				if (r < 38 && rightWasWhite)
				{
					rightAngles.add(dp.getAngleIncrement());
					rightWasWhite = false;
				}
				else if (r > 42)
				{
					rightWasWhite = true;
				}
			}
			if (suppressed)
				dp.stop();
			
			LCD.drawInt(leftAngles.size(), 0, 3);
			LCD.drawInt(rightAngles.size(), 0, 4);

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
				dp.rotate(a - previous);

				sonar.ping();
				int n = sonar.getDistances(echoes);
				int min = 255;
				for (j = 0; j < n; j++)
				{
					min = echoes[j] < min ? echoes[j] : min;	
				}

				min += AXLE_TO_SONAR/10;
				
				int globalHeading = Robot.toGlobal(Math.round(a/90));

				if (min < SONAR_TRUSTED_DISTANCE)
					Robot.getJunction().setEdge(globalHeading, Junction.Status.BLOCKED);
				else
					Robot.getJunction().setEdge(globalHeading, Junction.Status.FREE);

				previous = a;
			}
			
			if (path == null)
			{
				Sound.beep();
				LCD.drawString("Hello", 0, 0);
				Button.waitForAnyPress();
			}
			if (path.empty())
			{
				path = Robot.getJunction().pathToNearestUnknown();
				if (path == null)
				{
					exploring = false;
					Sound.beep();
					path = Robot.getJunction().pathTo(0, 0);
					return;
				}
				if (path.empty())
				{
					dp.rotate(-previous);
					dp.travel(-2*AXLE_TO_LIGHTS);
					left = leftLight.readValue();
					right = rightLight.readValue();
					return;
				}
			}
			
			int heading = path.pop();
			LCD.drawInt(heading, 4, 0);
			LCD.drawInt(Robot.toLocal(heading), 4, 1);
			float angle = 1080;
			for (int i = 0; i < angles.size(); i++)
			{
				if (Math.abs(angles.get(i) - Robot.toLocal(heading)*90) < 45)
				{
					angle = angles.get(i);
					break;
				}
			}
			
			dp.rotate(angle - previous);
			
			Robot.heading = heading;
			Robot.forward();
			
			left = leftLight.readValue();
			right = rightLight.readValue();
		}

		@Override
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
			}
		}

		@Override
		public void suppress() { suppressed = true; }
	};
	
	public static class Robot
	{
		public static int x = 0;
		public static int y = 0;
		public static int heading = 0;
		
		public static Junction getJunction() { return maze[x][y]; }
		public static int toGlobal(int heading) { return (heading + Robot.heading) % 4; }
		public static int toLocal(int heading) { return (heading - Robot.heading + 16) % 4; }
		
		public static void forward()
		{
			switch (heading)
			{
			case 0:
				y++;
				break;
			case 1:
				x--;
				break;
			case 2:
				y--;
				break;
			case 3:
				x++;
				break;
			default:
				Sound.beep();
			}
		}
		
		public static void backward()
		{
			switch (heading)
			{
			case 0:
				y--;
				break;
			case 1:
				x++;
				break;
			case 2:
				y--;
				break;
			case 3:
				x--;
				break;
			default:
				Sound.beep();
			}
		}
	}
}
