
package org.usfirst.frc.team6172.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.NetworkButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * This code runs our 2018 robot, currently nameless. We've implemented a threaded structure for throwing a cube
 * so that way the robot can continue to do other things while we are doing others.
 * 
 * 
 * @author Team 6172 The Mad Dogs
 *
 */
public class Robot extends IterativeRobot {
	
	//Our Talons (CAN Talons)
	//The first 4 are the drive train motors
	//The next 2 are the left and right side cube sucker motors
	//The next motor controller is for the arm
	//The final one is for the climber
	private WPI_TalonSRX frontLeftDT;
	private WPI_TalonSRX rearLeftDT;
	private WPI_TalonSRX frontRightDT;
	private WPI_TalonSRX rearRightDT;
	private WPI_TalonSRX leftCubeSucker;
	private WPI_TalonSRX rightCubeSucker;
	private WPI_TalonSRX arm;
	private WPI_TalonSRX climber; 
	
	//Our SpeedControllerGroups
	//The first two represent the motors on the left and right side of our drive train
	//The last one represents the motors on the left and right side of our cube sucking mechanism
	private SpeedControllerGroup leftDT;
	private SpeedControllerGroup rightDT;
	private SpeedControllerGroup cubeSucker;
	
	//Our DifferentialDrive instance, allowing us to control the robot using arcade or tank drive formats
	private DifferentialDrive dt;
	
	//Our Compressor object, while we do not have an onboard compressor on the robot as of this moment, we do
	//need to have this in order to charge our pneumatics before a match.
	private Compressor comp;
	
	//Our DoubleSolenoid object
	//we have one for the claw
	private DoubleSolenoid claw;
	
	//Our DigitalInput objects
	//The first is for detecting whether or not there is a cube in the claw
	private DigitalInput cubeIn;
	
	//Our XboxController objects
	//The naming should make this obvious, currently both controllers can do everything the robot
	//can do, with the exception that only the primary can drive
	//It may be better in the future to only allow the primary to drive and control the climber
	//and allow the secondary to do the rest, things may get complicated if both drivers
	//can do pretty much everything
	private XboxController primary;
	private XboxController secondary;
	
	private Timer autoTimer;
	
	private SendableChooser<String> autoChooser;
	
	private static final String AUTOLEFT = "Auto Position Left";
	private static final String AUTOCENTERGOLEFT = "Auto Position Center Turn Left";
	private static final String AUTOCENTERGORIGHT = "Auto Position Center Turn Right";
	private static final String AUTORIGHT = "Auto Position Right";
	
	@Override
	public void robotInit() {
		//Create all of our CAN based motor controllers
		frontLeftDT = new WPI_TalonSRX(3);
		rearLeftDT = new WPI_TalonSRX(4);
		frontRightDT = new WPI_TalonSRX(1);
		rearRightDT = new WPI_TalonSRX(2);
		leftCubeSucker = new WPI_TalonSRX(7);
		leftCubeSucker.setInverted(true); //Note this, we invert the direction of the left cube sucker, this will still maintain the inversion even in a speed controller group
		rightCubeSucker = new WPI_TalonSRX(5);
		arm = new WPI_TalonSRX(6);
		climber = new WPI_TalonSRX(8);
		
		//Create out speed controller groups for our left and right drive train sides and for our cubesucker
		leftDT = new SpeedControllerGroup(frontLeftDT, rearLeftDT);
		rightDT = new SpeedControllerGroup(frontRightDT, rearRightDT);
		cubeSucker = new SpeedControllerGroup(leftCubeSucker, rightCubeSucker);
		
		//Create our drivetrain object using the left and right side speed controller groups
		dt = new DifferentialDrive(leftDT, rightDT);
		
		//Create our compressor object
		comp = new Compressor();
		
		//Create our claw DoubleSolenoid object
		claw = new DoubleSolenoid(2, 3);
		
		//Create out limit switch digital input, using port 1
		cubeIn = new DigitalInput(1);
		
		//Create our 2 XboxController references, using USB 0 and 1 on the driver's station
		primary = new XboxController(0);
		secondary = new XboxController(1);
		
		autoTimer = new Timer();
		
		autoChooser = new SendableChooser<String>();
		autoChooser.addDefault(AUTOLEFT, AUTOLEFT);
		autoChooser.addObject(AUTOCENTERGOLEFT, AUTOCENTERGOLEFT);
		autoChooser.addObject(AUTOCENTERGORIGHT, AUTOCENTERGORIGHT);
		autoChooser.addObject(AUTORIGHT, AUTORIGHT);
		
		SmartDashboard.putData("Auto Selector", autoChooser);
	}

	/**
	 * Called once just before auto starts, nothing happens here at the moment...
	 */
	@Override
	public void autonomousInit() {
		autoTimer.reset();
		autoTimer.start();
	}

	/**
	 * Called every 20ms or so during auto, nothing happens here at the moment...
	 */
	@Override
	public void autonomousPeriodic() { 
		switch(autoChooser.getSelected())
		{
			case AUTOLEFT:
			case AUTORIGHT:
				if(autoTimer.get() < 4)
				{
					dt.tankDrive(1, 1);
				}
				else
				{
					dt.tankDrive(0, 0);
				}
				break;
			case AUTOCENTERGOLEFT:
				if(autoTimer.get() < 2)
				{
					dt.tankDrive(1, 1);
				}
				else if(autoTimer.get() >= 2 && autoTimer.get() < 3.5)
				{
					dt.tankDrive(-1, 1);
				}
				else if(autoTimer.get() >= 3.5 && autoTimer.get() < 6.5)
				{
					dt.tankDrive(1, 1);
				}
				else if(autoTimer.get() >= 6.5 && autoTimer.get() < 8)
				{
					dt.tankDrive(1, -1);
				}
				else if(autoTimer.get() >=8 && autoTimer.get() < 10)
				{
					dt.tankDrive(1, 1);
				}
				else
				{
					dt.tankDrive(0, 0);
				}
				break;
			case AUTOCENTERGORIGHT:
				if(autoTimer.get() < 2)
				{
					dt.tankDrive(1, 1);
				}
				else if(autoTimer.get() >= 2 && autoTimer.get() < 3.5)
				{
					dt.tankDrive(1, -1);
				}
				else if(autoTimer.get() >= 3.5 && autoTimer.get() < 6.5)
				{
					dt.tankDrive(1, 1);
				}
				else if(autoTimer.get() >= 6.5 && autoTimer.get() < 8)
				{
					dt.tankDrive(-1, 1);
				}
				else if(autoTimer.get() >= 8 && autoTimer.get() < 10)
				{
					dt.tankDrive(1, 1);
				}
				else
				{
					dt.tankDrive(0, 0);
				}
				break;
		}
		
	}
	
	/**
	 * Called when teleop first starts, it starts the compressor
	 */
	public void teleopInit()
	{
		comp.start();
	}

	/**
	 * Our teleop periodic method, called every 20ms to make our teleop
	 * stuff work, there are 4 big pieces, making the cube sucker work
	 * make the arm and throwing work, making the climber work, and driving
	 * Driving is the only element directly implemented here, there rest happens
	 * in methods defined below
	 */
	@Override
	public void teleopPeriodic() {
		//Call arcade drive, using the left hand y and x axis' from the
		//primary driver's xbox controller to make the robot move
		dt.arcadeDrive(primary.getY(Hand.kLeft), primary.getX(Hand.kLeft));
		
		//Check our cube sucker stuff
		cubeSuckerOI();
		
		//Check out arm throwing stuff
		armAndThrowingOI();
		
		//Check the climber stuff
		climberOI();
	}

	/**
	 * Called once when the robot is first disabled, we just make sure the compressor
	 * is stopped here, while the compressor won't run unless the robot is enabled
	 * this should prevent closed loop control from continuing if code other than
	 * this is uploaded
	 */
	public void disabledInit()
	{
		comp.stop();
	}
	
	/**
	 * This method is to be called in teleopPeriodic, it allows you to control
	 * the cube sucker, the button map is as follows:
	 * 
	 * Primary or Secondary/Button A = Suck cube in if there isn't one already there
	 * Primary or Secondary/Left Bumper = Push cube out
	 * Primary or Secondary/Right Bumper = Open claw
	 * 
	 * Note that the buttons above only work if the robot is not currently in 
	 * the process of throwing a cube
	 */
	private void cubeSuckerOI()
	{
		if(primary.getBumper(Hand.kRight) || secondary.getBumper(Hand.kRight))
		{
			claw.set(DoubleSolenoid.Value.kReverse);
		}
		
		//If the A button is pressed on either controller
		if(primary.getAButton() || secondary.getAButton())
		{
			claw.set(DoubleSolenoid.Value.kForward);
			
			//If we do not currently have a cube, set the motors to suck in a cube
			if(!cubeIn.get()){
				cubeSucker.set(1);
			}
			else //Otherwise stop the motors
			{
				cubeSucker.set(0);
			}
		}
		//If the left bumper is pressed on either controller
		else if(primary.getBumper(Hand.kLeft) || secondary.getBumper(Hand.kLeft))
		{
			//Reverse the motors so it shoots out a cube
			cubeSucker.set(-1);
		}
		else //Otherwise, if there is no buttons being pressed, stop the motors
		{
			cubeSucker.set(0);
		}
	}
	
	/**
	 * This method is to be called in teleopPeriodic, it allows you to control
	 * the arm and the process of throwing a cube (at least in part), the 
	 * button map is as follows:
	 * 
	 * Primary or Secondary/Button X = Bring the arm down
	 * Primary or Secondary/Button B = Bring the arm up
	 * 
	 * Note that the buttons above only work if the robot is not currently in
	 * the process of throwing a cube
	 */
	private void armAndThrowingOI()
	{
		//If the X button is pressed on either controller run the arm down
		if(primary.getXButton() || secondary.getXButton())
		{
			arm.set(1);
		}
		//Otherwise, if the B button is pressed on either controller
		else if(primary.getBButton() || secondary.getBButton())
		{
			arm.set(-1);
		}
		//Otherwise, disable the arm motor
		else
		{
			arm.set(0);
		}
	}
	
	/**
	 * This method is to be called in teleopPeriodic, it allows you to control
	 * the climber, the button map is as follows:
	 * 
	 * Primary or Secondary/Y Button = Start to climb
	 */
	private void climberOI()
	{
		//If the y button is pressed on either controller run the climber
		if((primary.getYButton() || secondary.getYButton()))
		{
			climber.set(-1);
		}
		//Otherwise, just stop the climber motor
		else
		{
			climber.set(0);
		}
	}
}
