package org.usfirst.frc.team695.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
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
5 left bump test chane
6 right bump
7 multi screen
8 options test changwe
9 left push
10 right push abcderg

*/

public class Robot extends SampleRobot
{
	
	// network communications
	private NetworkTableInstance inst;
	private NetworkTable table;
	private NetworkTableEntry xerr;
	private NetworkTableEntry ringop;
	private NetworkTableEntry tabhatchleft;
	private NetworkTableEntry tabhatchright;
	private NetworkTableEntry autopilot;
	private NetworkTableEntry hatchstatus;
	private NetworkTable limeLightValues;
	private NetworkTableEntry limeTx;
	private NetworkTableEntry limeTy;
	private NetworkTableEntry limeTa;

	// pneumatic objects
	private Compressor comp = new Compressor(0);
	private Solenoid hatch = new Solenoid(0);	// #1
	private Solenoid lift = new Solenoid(1);	// #2
	private Solenoid forks = new Solenoid(2);	// #3

	// user controller objects
//	private Joystick controllerAux = new Joystick(3);
	private Joystick controllerDrive = new Joystick(0);
	
	// lidar distance
	private Counter lidar1 = new Counter(9);

	// jack rotation counter
	//private Counter countJack = new Counter();

	// jack string potentiometer
	private AnalogInput jackpos = new AnalogInput(0);

	// jack in limit switch
	//private DigitalInput jackin = new DigitalInput(1);
	
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

		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(160, 120);
		camera.setExposureManual(50);
		//camera.setBrightness(50);
		

		inst = NetworkTableInstance.getDefault();
		table = inst.getTable("SmartDashboard");
		xerr = table.getEntry("xerr");
		ringop = table.getEntry("ringop");
		tabhatchleft = table.getEntry("tabhatchleft");
		tabhatchright = table.getEntry("tabhatchright");
		autopilot = table.getEntry("autopilot");
		hatchstatus = table.getEntry("hatchstatus");
		limeLightValues = inst.getTable("limelight");
		limeTx = limeLightValues.getEntry("tx");
		limeTy = limeLightValues.getEntry("ty");
		limeTa = limeLightValues.getEntry("ta");
		comp.enabled();

		//countJack.setUpSource(0);
		//countJack.setUpDownCounterMode();
		//System.out.println("count: " + countJack.get());
		
		hatch.set(false);
		lift.set(false);
		forks.set(false);

		lidar1.setMaxPeriod(1.0);
	    lidar1.setSemiPeriodMode(true);
	    lidar1.reset();

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
				forks.set(false);
				lift.set(false);
				hatch.set(false);
				hatchstatus.setNumber(0);
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
		long forksdebounce = 0;

		long headstand = 0;
		
		long tickcnt = 0;

		double mgain = 0.5;
		
		double err;
		
		double driveleft = 0;
		double driveright = 0;
	//	double drivesteer;
		
		double movejack;
		double x;// = limeTx.getDouble(0.0);
		double y;// = limeTy.getDouble(0.0);
		double area;// = limeTa.getDouble(0.0);
		double Kp = 0.03;  // Proportional control constant
		double steeringAdjust = 0;
		double minCommand = -0.015;
		System.out.println("695:  operatorControl()...");
		System.out.println("Ring is green!");
		ringop.setNumber(3);
		//countJack.reset();

		while(isEnabled())
		{

			if ((controllerDrive.getPOV() == 180) && (mgain > 0.1))
			{
				mgain = mgain - 0.1;
			}
			if ((controllerDrive.getPOV() == 0) && (mgain < 1))
			{
				mgain = mgain + 0.1;
			}
			System.out.println("reachrf");
			x = limeTx.getDouble(0.0);
			y = limeTy.getDouble(0.0);
			area = limeTa.getDouble(0.0);

			System.out.println("LIME DATA: X: " + Double.toString(x) + " Y: " + Double.toString(y) + " AREA: " + Double.toString(area));
			/* jack move
			movejack = controllerDrive.getPOV();

			if (movejack == -1)
			{
				motorJack.set(ControlMode.PercentOutput, 0);
			}

			// jack moving in
			else
			{

				if (movejack == 180)
				{
					if (jackpos.getValue() <= 3800)
					{
						motorJack.set(ControlMode.PercentOutput, 1);
					}
					else
					{
						motorJack.set(ControlMode.PercentOutput, 0);
					}
				}
			
				// jack moving out
				if (movejack == 0)
				{
					// if in headstand position, lowwer lift and pull forks back in as jack rises
					if (headstand == 1)
					{
						if (jackpos.getValue() < 3000) // ????
						{
							lift.set(false);
							forks.set(false);
						}
					}
					if (jackpos.getValue() >= 2200)
					{
						motorJack.set(ControlMode.PercentOutput, -1);
						movejack = 0;
					}
					else
					{
						motorJack.set(ControlMode.PercentOutput, 0);
					}
				}
			}
			*/

			// hatch lift up/down
			if (controllerDrive.getRawButton(1) == true)
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
			
			// hatch auto grab
			if ((hatchleft.get() == true) && (hatchright.get() == true))
			{
				hatch.set(true);
				hatchstatus.setNumber(1);
			}

			// hatch manual grab/release
			if (controllerDrive.getRawButton(6) == true)
			{
				if (hatchdebounce == 0)
				{
					hatchdebounce = 1;
					if (hatch.get() == true)
					{
						hatch.set(false);
						hatchstatus.setNumber(0);
					}
					else
					{
						hatch.set(true);
						hatchstatus.setNumber(1);
					}
				}
			}
			else
			{
				hatchdebounce = 0;
			}
			
			// update network tables for UI display
			if (hatchleft.get() == true)
			{
				tabhatchleft.setNumber(1);
			}
			else
			{
				tabhatchleft.setNumber(0);
			}
			
			// update network tables for UI display
			if (hatchright.get() == true)
			{
				tabhatchright.setNumber(1);
			}
			else
			{
				tabhatchright.setNumber(0);
			}

			/* headstand
			if (controllerDrive.getRawButton(1) == true)
			{
				if (forksdebounce == 0)
				{
					forksdebounce = 1;
					if (forks.get() == true)
					{
						forks.set(false);
						headstand = 0;
					}
					else
					{
						forks.set(true);
						headstand = 1;
					}
				}
			}
			else
			{
				forksdebounce = 0;
			}
			*/


			
			//***********
			// drive code
			//***********

			// check for auto move away from habitat wall

			/*
			if (controllerAux.getRawButton(1) == true)
			{
				drivesteer = 0;

				if (getLidar() < 20)
				{
					driveleft = driveright = -0.15;
				}
				else
				{
					driveleft = driveright = 0.05;

					if (controllerAux.getRawButton(2) == true)
					{
						if (forksdebounce == 0)
						{
							forksdebounce = 1;
							if (forks.get() == true)
							{
								forks.set(false);
								headstand = 0;
							}
							else
							{
								forks.set(true);
								headstand = 1;
							}
						}
					}
					else
					{
						forksdebounce = 0;
					}
					
		
				}
			}
			*/

			/*
			if (1 == 0)
			{

			}
			
			// otherwise, normal driving
			else
			{
			*/
			double forwardModifier = 0;
			driveleft  = controllerDrive.getRawAxis(5);
			driveright = controllerDrive.getRawAxis(1);
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
				driveleft = driveright; //disable tank drive by ignoring right stick, left becomes the throttle
				driveleft += steeringAdjust;
				driveright -= steeringAdjust;
				driveright += forwardModifier;
				driveleft += forwardModifier;
				// drive speed
			}
				

			/*	if ((driveleft > 0.1) && (driveleft < -0.1))
				{
					driveleft = driveright = 0;
				}
			{}*/
				// autopilot steering
				//err = xerr.getDouble(0) / 100;

				//if (controllerDrive.getRawButton(5) == true)
				//{
					//autopilot.setNumber(1);

				//	drivesteer = err;

				//	if (driveleft > 0.25)
				//	{
				//		driveleft = driveright = 0.25;
				//	}
				//	if (driveleft < -0.25)
				//	{
				//		driveleft = driveright = -0.25;
				//	}

				//}
				//else
				//{
				//	autopilot.setNumber(0);
				//}
			
			
			
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
		/*		if (drivesteer > 0)
				{
					driveleft = driveleft * (1 - drivesteer);
				}
				if (drivesteer < 0)
				{
					driveright = driveright * (1 + drivesteer);
				}

			} */

			driveleft = mgain * driveleft;
			driveright = mgain * driveright;

			motorL1.set(ControlMode.PercentOutput, driveleft);
			motorL2.set(ControlMode.PercentOutput, driveleft);

			motorR1.set(ControlMode.PercentOutput, -1 * driveright);
			motorR2.set(ControlMode.PercentOutput, -1 * driveright);
		}
		
		}

						
			//******************************
			// diagnostic print every second
			//******************************
	/*		if (++tickcnt == 100)
			{
				//System.out.println(controllerDrive.getRawAxis(0));
				//System.out.println(controllerDrive.getRawAxis(4));
				//System.out.println("");
				tickcnt = 0;
				//System.out.println("POV: " + controllerDrive.getPOV());
				//System.out.println("raw button 2: " + controllerDrive.getRawButton(2));
				System.out.println("driveleft=" + driveleft + ", driveright=" + driveright + ", drivesteer=" + drivesteer);
				//System.out.println("ERR: " + err + ", GAIN: " + pgain);
				//System.out.println("JACK: " + countJack.get());
				//System.out.println("695:  operatorControl(" + (++cnt) + ")");
				//System.out.println(hatchleft.get() + " / " + hatchright.get());
				//System.out.println("hatch: " + hatch.get() + ":  " + hatchleft.get() + " / " + hatchright.get());
				//System.out.println("   jack distance: " + getLidar());
				//if (forks.get() == true)
				//{
			//		System.out.println("   * * * HEADSTAND * * *");
		//		}
				//System.out.println("   jack in: " + jackin.get());
				//System.out.println("   movejack: " + movejack);
				//System.out.println("jackpos: " + jackpos.getValue());
		
			}
			
			//***********************
			// time delay for roborio
			//***********************
			Timer.delay(0.025);
			
		}		
	}
	*/
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
		
