package org.usfirst.frc.team695.robot;





import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.cameraserver.CameraServer;

/*
XBOX controls:

axis:
0 left X
1 left Y
2 left trigger
3 right trigger
4 right X
5 right Y

buttons:
1 A
2 B
3 X
4 Y
5 left bump
6 right bump
7 multi screen
8 options
9 left push
10 right push

*/

public class Robot extends SampleRobot
{
	
	// network communications
	private NetworkTableInstance inst;
	private NetworkTable table;
	private NetworkTableEntry pidx;
	private NetworkTableEntry ringop;
	private NetworkTableEntry tabhatchleft;
	private NetworkTableEntry tabhatchright;
	private NetworkTable limeLightValues;
	private NetworkTableEntry limeTx;
	private NetworkTableEntry limeTy;
	private NetworkTableEntry limeTa;
	// pneumatic objects
	private Compressor comp = new Compressor(0);
	private Solenoid hatch = new Solenoid(0);
	private Solenoid lift = new Solenoid(1);
	private Solenoid forks = new Solenoid(2);

	// user controller objects
	private Joystick controllerLiftJack = new Joystick(3);
	private Joystick controllerDrive = new Joystick(0);
	
	// lidar distance
	private Counter lidar1 = new Counter(9);

	// jack rotation counter
	private Counter countJack = new Counter();

	// jack in limit switch
	private DigitalInput jackin = new DigitalInput(1);
	
	// hatch detection switches
	private DigitalInput hatchleft = new DigitalInput(6);
	private DigitalInput hatchright = new DigitalInput(5);
	
	// motor controllers
	private VictorSPX motorL1 = new VictorSPX(1);
	private VictorSPX motorL2 = new VictorSPX(2);
	private VictorSPX motorR1 = new VictorSPX(3);
	private VictorSPX motorR2 = new VictorSPX(4);
	private VictorSPX motorJack = new VictorSPX(8);


	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	public void leds()
	{
	}
	
	
	public double regulate(double speed, long elevatorpos)
	{
		if ((elevatorpos >= 1250) && (elevatorpos < 3750))
		{
			if (speed > 0.6)
			{
				speed = 0.6;
			}
			else if (speed < -0.6)
			{
				speed = -0.6;
			}
		}
		if (elevatorpos >= 3750)
		{
			if (speed > 0.3)
			{
				speed = 0.3;
			}
			else if (speed < -0.3)
			{
				speed = -0.3;
			}
		}
		return(speed);
	}
	
	// function getLidar() returns distance in inches
	public double getLidar()
	{
		if (lidar1.get() < 1)
		{
			System.out.println("Lidar connected to DIO9?");
			return 0;
		}
		/* getPeriod returns time in seconds. The hardware resolution is microseconds.
		 * The LIDAR-Lite unit sends a high signal for 10 microseconds per cm of distance.
		 */
		
		return((lidar1.getPeriod() * 1000000.0 / 10.0) / 2.54);
	}

	
	
	
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	public String getAlliance()
	{
		DriverStation.Alliance color = DriverStation.getInstance().getAlliance();
		
		if (color == DriverStation.Alliance.Blue)
		{
			return("B");
			//return("BLUE" + DriverStation.getInstance().getLocation());
		}
		else if (color == DriverStation.Alliance.Red)
		{
			return("R");
		}
		return("?");
	}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	public void robotInit()
	{
		System.out.println("695:  robotInit()");

		//UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		//camera.setResolution(160, 120);

		inst = NetworkTableInstance.getDefault();
		table = inst.getTable("SmartDashboard");
		pidx = table.getEntry("pidx");
		ringop = table.getEntry("ringop");
		tabhatchleft = table.getEntry("tabhatchleft");
		tabhatchright = table.getEntry("tablehatchright");
		limeLightValues = inst.getTable("limelight");
		limeTx = limeLightValues.getEntry("tx");
		limeTy = limeLightValues.getEntry("ty");
		limeTa = limeLightValues.getEntry("ta"); 		
		comp.enabled();

		countJack.setUpSource(0);
		countJack.setUpDownCounterMode();
		System.out.println("count: " + countJack.get());
		
		forks.set(false);

		//lidar1.setMaxPeriod(1.0);
	    //lidar1.setSemiPeriodMode(true);
	    //lidar1.reset();

		/*
		CameraServer.getInstance().startAutomaticCapture();
		
		
		System.out.println("695:  robotInit():  camera");
		new Thread(() ->
		{
           UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
           camera.setResolution(320, 240);
           camera.setFPS(10);
           
           CvSink cvSink = CameraServer.getInstance().getVideo();
           CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 320, 240);
           
           Mat source = new Mat();
           Mat output = new Mat();
           
           while(!Thread.interrupted())
           {
               cvSink.grabFrame(source);
               Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
               outputStream.putFrame(output);
           }
       }
		).start();
		*/	
		
	}

	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	public void disabled()
	{		
		long cnt = 0;
		long tickcnt = 0;
		String ringstr;
		
		for(;;)
		{
				
			if (getAlliance() == "R")
			{
				ringop.setNumber(1);
				ringstr = "RED";
			}
			else
			{
				ringop.setNumber(2);
				ringstr = "BLUE";
			}

			if (!isDisabled())
			{
				return;
			}
			
			if (++tickcnt == 100)
			{
				System.out.println("695:  disabled(" + (++cnt) + "):  Alliance is " + ringstr);
				tickcnt = 0;
			}

			Timer.delay(0.01);

		}
		
		//}
	}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	public void autonomous()
	{
		System.out.println("695:  autonomous()");
		operatorControl();
	}

	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	public void operatorControl()
	{
		long hatchdebounce = 0;
		long liftdebounce = 0;
		long povdebounce = 0;
		long forksdebounce = 0;
		
		long tickcnt = 0;
		
		double err, pgain = 0.3;
		
		double driveleft;
		double driveright;
		double drivesteer;
		
		double x;// = limeTx.getDouble(0.0);
		double y;// = limeTy.getDouble(0.0);
		double area;// = limeTa.getDouble(0.0);
		double Kp = 0.03;  // Proportional control constant
		double steeringAdjust = 0;
		double minCommand = -0.015;


		double movejack;

		System.out.println("695:  operatorControl()...");
		System.out.println("Ring is green!");
		ringop.setNumber(3);
		countJack.reset();

		while(isEnabled())
		{
			// *********
			// jack code
			// *********
			
			if (controllerLiftJack.getRawButton(2) == true)
			{
				if (forksdebounce == 0)
				{
					forksdebounce = 1;
					if (forks.get() == true)
					{
						forks.set(false);
					}
					else
					{
						forks.set(true);
					}
				}
			}
			else
			{
				forksdebounce = 0;
			}
			
			movejack = controllerLiftJack.getRawAxis(1);
			if ((movejack >= -0.5) && (movejack <= 0.5))
			{
				movejack = 0;
			}
			
			if (jackin.get() == true)
			{
				countJack.reset();
			}
			
			// jack moving in
			if (movejack > 0)
			{
				if (jackin.get() == true)
				{
					movejack = 0;
				}
			}
			
			// jack moving out
			if (movejack < 0)
			{
				if (countJack.get() >= 470)
				{
					movejack = 0;
				}
			}
			motorJack.set(ControlMode.PercentOutput, movejack);
						
			if ((hatchleft.get() == true) && (hatchright.get() == true))
			{
				hatch.set(true);
			}

			if (controllerDrive.getPOV() != -1)
			{
				if (povdebounce == 0)
				{
					povdebounce = 1;
					if (controllerDrive.getPOV() == 0)
					{
						if (pgain < 1)
						{
							pgain = pgain + 0.1;
						}
					}
					if (controllerDrive.getPOV() == 180)
					{
						if (pgain > 0.1)
						{
							pgain = pgain - 0.1;
						}
					}
				}
			}
			else
			{
				povdebounce = 0;
			}
									
			// lift
			if (controllerDrive.getRawButton(2) == true)
			{
				if (liftdebounce == 0)
				{
					liftdebounce = 1;
					if (lift.get() == true)
					{
						lift.set(false);
					}
					else
					{
						lift.set(true);
					}
				}
			}
			else
			{
				liftdebounce = 0;
			}
			
			// hatch
			if (controllerDrive.getRawButton(8) == true)
			{
				if (hatchdebounce == 0)
				{
					hatchdebounce = 1;
					if (hatch.get() == true)
					{
						hatch.set(false);
					}
					else
					{
						hatch.set(true);
					}
				}
			}
			else
			{
				hatchdebounce = 0;
			}
			
			if (hatchleft.get() == true)
			{
				tabhatchleft.setNumber(1);
			}
			else
			{
				tabhatchleft.setNumber(0);
			}
			
			if (hatchright.get() == true)
			{
				tabhatchright.setNumber(1);
			}
			else
			{
				tabhatchright.setNumber(0);
			}
			
			//***********
			// drive code
			//***********

			// drive speedn
			driveleft = driveright = controllerDrive.getRawAxis(1);
			drivesteer = controllerDrive.getRawAxis(4);

			if ((driveleft >= -0.1) && (driveleft <= 0.1))
			{
				driveleft = driveright = 0;
			}
			
			// auto dock
			err = pidx.getDouble(0) / 100;

			if (controllerDrive.getRawButton(7) == true)
			{

				drivesteer = err * pgain;

				if (driveleft > 0.25)
				{
					driveleft = driveright = 0.25;
				}
				if (driveleft < -0.25)
				{
					driveleft = driveright = -0.25;
				}

			}
			
			

			// check for dime turn drive
		//	if ((driveleft >= -0.1) && (driveleft <= 0.1))
		//	{
		//		drivesteer *= 0.4;
		//		driveleft = -1 * drivesteer;
		//		driveright = drivesteer;
		//	}
			
			// otherwise drive normally forward / backwards
		//	else
		//	{
				
				// apply steering to move
				if (drivesteer > 0)
				{
					driveleft = driveleft * (1 - drivesteer);
				}
				if (drivesteer < 0)
				{
					driveright = driveright * (1 + drivesteer);
				}

				// adjust speed if elevator raised
				driveleft = regulate(driveleft, 0);
				driveright = regulate(driveright, 0);
				
		//	}
		x = limeTx.getDouble(0.0);
		y = limeTy.getDouble(0.0);
		area = limeTa.getDouble(0.0);

		System.out.println("LIME DATA: X: " + Double.toString(x) + " Y: " + Double.toString(y) + " AREA: " + Double.toString(area));
		double forwardModifier = 0;
		//driveleft  = controllerDrive.getRawAxis(5);
		//driveright = controllerDrive.getRawAxis(1);
		if (controllerDrive.getRawButton(3)) {
			System.out.println("PBUTTON DOWN");
			steeringAdjust = Kp*x;
			if (x > 3.0)
			{
				steeringAdjust = Kp*x - minCommand;
			}
			else if (x < 3.0)
			{
				steeringAdjust = Kp*x + minCommand;
				forwardModifier = 0;//-0.5;
			}
			//driveleft = driveright; //disable tank drive by ignoring right stick, left becomes the throttle
			driveleft += steeringAdjust;
			driveright -= steeringAdjust;
			driveright += forwardModifier;
			driveleft += forwardModifier;
			// drive speed
		}
			
			motorL1.set(ControlMode.PercentOutput, driveleft);
			motorL2.set(ControlMode.PercentOutput, driveleft);

			motorR1.set(ControlMode.PercentOutput, -1 * driveright);
			motorR2.set(ControlMode.PercentOutput, -1 * driveright);

						
			//******************************
			// diagnostic print every second
			//******************************
			if (++tickcnt == 100)
			{
				tickcnt = 0;
				//System.out.println("raw button 2: " + controllerDrive.getRawButton(2));
				//System.out.println("driveleft=" + driveleft + ", driveright=" + driveright + ", drivesteer=" + drivesteer);
				System.out.println("GAIN: " + pgain);
				//System.out.println("JACK: " + countJack.get());
				//System.out.println("695:  operatorControl(" + (++cnt) + ")");
				//System.out.println(hatchleft.get() + " / " + hatchright.get());
				//System.out.println("hatch: " + hatch.get() + ":  " + hatchleft.get() + " / " + hatchright.get());
				//System.out.println("   jack count: " + countJack.get());
				//System.out.println("   jack distance: " + getLidar());
				//System.out.println("   jack in: " + jackin.get());
				//System.out.println("   movejack: " + movejack);
				leds();
		
			}
			
			//***********************
			// time delay for roborio
			//***********************
			Timer.delay(0.01);
			
		}		
	}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	public void test()
	{
		long cnt = 0;

		System.out.println("695:  test()");
		while (isTest() && isEnabled())
		{
			Timer.delay(1);
			System.out.println("695:  test tick() " + (++cnt));
		}
	}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

}



/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/
/**************************************************************************************************************************************************/

//public class Robot extends IterativeRobot
//{
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	//public void robotInit()
	//{
		//System.out.println("robotInit()");		
	//}

	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	//public void disabledInit()
	//{
		//System.out.println("disabledInit()");
	//}
	
	//public void disabledPeriodic()
	//{
		
	//}
	
	//public void disabledContinuous()
	//{
		
	//}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/

	//public void autonomousInit()
	//{
		//System.out.println("autonomousInit()");
	//}
	
	//public void autonomousPeriodic()
	//{
		
	//}

	//public void autonomousContinuous()
	//{
		
	//}

	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
	
	//public void teleopInit()
	//{
		//System.out.println("teleopInit()");		
	//}
	
	//public void teleopPeriodic()
	//{
		
	//}
	
	//public void teleopContinuous()
	//{
		
	//}
	
	/****************************************************************/
	/****************************************************************/
	/****************************************************************/
		
