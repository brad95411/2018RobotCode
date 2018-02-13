
package org.usfirst.frc.team6172.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.NetworkButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
	
	//Our DoubleSolenoid objects, we have one for the claw and one for the idler pulley. 
	private DoubleSolenoid claw;
	private DoubleSolenoid idlerPulley;
	
	//Our Servo object, used for pulling the climber pin, this may change later, but the Servo
	//provides for a example to make the code complete
	private Servo climberPin;
	
	//Our DigitalInput objects
	//The first one is for our throw release, when the arm reaches a certain position, this switch
	//will be used to detect when to let the cube go
	//The second one is for detecting when the arm is all the way down
	//The third one is for detecting whether or not there is a cube in the claw
	private DigitalInput throwReleaseSwitch;
	private DigitalInput armDownSwitch;
	private DigitalInput cubeIn;
	
	//Our XboxController objects
	//The naming should make this obvious, currently both controllers can do everything the robot
	//can do, with the exception that only the primary can drive
	//It may be better in the future to only allow the primary to drive and control the climber
	//and allow the secondary to do the rest, things may get complicated if both drivers
	//can do pretty much everything
	private XboxController primary;
	private XboxController secondary;
	
	//Our NetworkButton objects
	//NetworkButtons are special objects that put buttons on the SmartDashboard, 
	//The first button is available to reset the servo when we are in between 
	//matches using an Ethernet cable
	//The second button acts as an emergency stop if something goes wrong with the
	//cube throwing thread
	private NetworkButton resetServo;
	private NetworkButton throwEStop;
	
	//Our Thread object
	//A little background on Threads, a thread is something that runs separately from the rest of the code
	//to complete an action, CPUs run different threads one after the other after the other
	//very quickly (very very quickly) to simulate things running at the same time. The objective of 
	//this thread is to automate the process of throwing a cube once you've pressed a button
	private Thread throwThread;
	
	//Our boolean primitives
	//The first one indicates to the code in different spots that we are currently trying to throw a cube
	//The second one indicates to the code in different spots that we are ready to climb (i.e. the climber is deployed)
	private boolean throwing;
	private boolean readyToClimb;
	
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
		
		//Create our claw and idler pulley DoubleSolenoid objects
		claw = new DoubleSolenoid(0, 1);
		idlerPulley = new DoubleSolenoid(2, 3);
		
		//Create our servo object and indicate that it is connected to PWM 0
		climberPin = new Servo(0);
		
		//Create out limit switch digital inputs, using ports 0, 1, and 5 on the DIO rail
		armDownSwitch = new DigitalInput(0);
		cubeIn = new DigitalInput(1);
		throwReleaseSwitch = new DigitalInput(5);
		
		//Create our 2 XboxController references, using USB 0 and 1 on the driver's station
		primary = new XboxController(0);
		secondary = new XboxController(1);
		
		//Create a SmartDashboard buttons that can be used to reset the climber pin servo back to it's original position
		//and another button to act as an emergency stop for the throwing thread
		resetServo = new NetworkButton("SmartDashboard", "Reset Servo");
		throwEStop = new NetworkButton("SmartDashboard", "Throwing E-Stop");
	}

	/**
	 * Called once just before auto starts, nothing happens here at the moment...
	 */
	@Override
	public void autonomousInit() {
	}

	/**
	 * Called every 20ms or so during auto, nothing happens here at the moment...
	 */
	@Override
	public void autonomousPeriodic() { 
		
	}
	
	/**
	 * Called when teleop first starts, starts the compressor and sets 
	 * up our throwing and ready to climb variables
	 */
	public void teleopInit()
	{
		comp.start();
		throwing = false;
		readyToClimb = false;
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
	 * 
	 * Note that the buttons above only work if the robot is not currently in 
	 * the process of throwing a cube
	 */
	private void cubeSuckerOI()
	{
		//If the A button is pressed on either controller and the 
		//robot is not throwing a cube
		if((primary.getAButton() || secondary.getAButton()) && !throwing)
		{
			//If we do not currently have a cube, set the motors to suck in a cube
			if(!cubeIn.get()){
				cubeSucker.set(1);
			}
			else //Otherwise stop the motors
			{
				cubeSucker.set(0);
			}
		}
		//If the left bumper is pressed on either controller and the robot is not throwing a cube
		else if((primary.getBumper(Hand.kLeft) || secondary.getBumper(Hand.kLeft)) & !throwing)
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
	 * Primary or Secondary/Button X = Bring the arm down if it's not already all the way down
	 * Primary or Secondary/Button B = Bring the arm up if it's not already all the way up
	 * Primary or Secondary/Right Bumper = Start the automatic cube throwing subroutine
	 * 
	 * Note that the buttons above only work if the robot is not currently in
	 * the process of throwing a cube
	 */
	private void armAndThrowingOI()
	{
		//Put whether or not we are currently throwing a cube on the 
		//SmartDashboard/ShuffleBoard
		//I do not currently condone the use of ShuffleBoard, it seems quite
		//buggy and unstable at the current moment, SmartDashboard is good enough
		//to get the job done, albeit it's somewhat ugly
		SmartDashboard.putBoolean("Currently Throwing?", throwing);
		
		//If we are throwing and the throw e stop network button is 
		//pressed, interrupt the throw thread and set it to null, hopefully
		//stopping any failed throw attempts
		if(throwing && throwEStop.get())
		{
			throwThread.interrupt();
			throwThread = null;
		}
		
		//If the X button is pressed on either controller and we are 
		//not currently throwing a cube
		if((primary.getXButton() || secondary.getXButton()) && !throwing)
		{
			//Ensure the idler pulley is locked in place
			idlerPulley.set(DoubleSolenoid.Value.kForward);
			
			//If the arm is not all the way down, set the motor to start
			//moving down
			if(!armDownSwitch.get())
			{
				arm.set(-1);
			}
			else //Otherwise stop the arm
			{
				arm.set(0);
			}
		}
		//Otherwise, if the B button is pressed on either controller and we are
		//not currently throwing a cube
		else if((primary.getBButton() || secondary.getBButton()) && !throwing)
		{
			//Ensure the idler pulley is locked in place
			idlerPulley.set(DoubleSolenoid.Value.kForward);
			
			//If the arm is not all the way up, set the motor to start
			//moving up
			if(!throwReleaseSwitch.get())
			{
				arm.set(1);
			}
			else //Otherwise stop the arm
			{
				arm.set(0);
			}
		}
		//Otherwise, if the right bumper is pressed on either controller and we are
		//currently not throwing a cube
		else if((primary.getBumper(Hand.kRight) || secondary.getBumper(Hand.kRight)) && !throwing)
		{
			//Set the throwing variable true to prevent manual motion of the 
			//cube sucker and arm
			throwing = true;
			
			//If we still have an active thread for some reason
			if(throwThread != null)
			{
				//Force the thread to join, doing nothing if there is a problem
				//and then set the throw thread variable to null
				try {
					throwThread.join();
				} catch (InterruptedException e) {
				} finally {
					throwThread = null;
				}
			}
			
			//Set throw thread to a new instance of a thread, using a 
			//new throwing runnable object (defined below) to make
			//the robot throw automatically and start the thread
			throwThread = new Thread(new ThrowingRunnable());
			throwThread.start();
		}
		//Otherwise, as long as we are not throwing, lock the arm in place
		//using the idler pulley and set the arm to not move
		else if(!throwing)
		{
			idlerPulley.set(DoubleSolenoid.Value.kForward);
			arm.set(0);
		}
	}
	
	/**
	 * This method is to be called in teleopPeriodic, it allows you to control
	 * the climber, the button map is as follows:
	 * 
	 * Primary or Secondary/Start Button = Pull the pin to put the climber arm up
	 * Primary or Secondary/Y Button = Start to climb
	 * 
	 * Note the climb button (Y) does not work at all unless you have pulled the pin
	 */
	private void climberOI()
	{
		//If the start button is pressed on either controller and the robot is
		//not ready to climb yet
		if((primary.getStartButton() || secondary.getStartButton()) && !readyToClimb)
		{
			//Set the climber pin servo to 90 degrees and indicate to the program
			//as a whole that the robot is ready to climb
			climberPin.setAngle(90);
			readyToClimb = true;
		}
		//Otherwise, if the reset servo button on the SmartDashboard is pressed
		//set the climber pin servo angle back to 0
		else if(resetServo.get())
		{
			climberPin.setAngle(0);
		}
		//Otherwise, if the Y button is pressed on either controller and the
		//robot is ready to climb, start the climber motor
		else if((primary.getYButton() || secondary.getYButton()) && readyToClimb)
		{
			climber.set(-1);
		}
		//Otherwise, just stop the climber motor
		else
		{
			climber.set(0);
		}
	}
	
	/**
	 * A private inner classes which takes some of the variables
	 * defined in the encompassing class to automatically throw a cube
	 * This is a bit experimental, but with some tweaking if the throwing
	 * stuff works mechanically this could be tweaked to work very easily
	 * 
	 * A Runnable is anything that be "run". Multiple runnables can be run
	 * alongside each other in things called Threads, a Thread is something that
	 * can run near simultaneously on a single processor core by quickly
	 * switching between which Thread is running at any given time.
	 * This allows for other parts of the code to continue running
	 * while we are throwing, for example, if you found a reason to drive
	 * the robot while throwing, you still could. 
	 * 
	 * @author Team 6172 The Mad Dogs
	 *
	 */
	private class ThrowingRunnable implements Runnable
	{
		/**
		 * The only method that is needed for a runnable.
		 * THIS METHOD IS ONLY CALLED ONCE PER THREAD.
		 * Internal loops keep the process going, once this method
		 * is finished nothing else happens, this won't rerun 
		 * unless you create a new thread with it.
		 */
		@Override
		public void run() {
			
			//While the arm is not all the way down
			while(!armDownSwitch.get()) {
				//Try to move the arm down
				arm.set(-1);
				
				//Try to reduce the update speed of the thread
				//so it doesn't take up all the system resources
				//If we have been interrupted (told to stop running)
				//then break out of the loop
				try {
					Thread.sleep(15); //Sleep uses milliseconds as a parameter
				} catch (InterruptedException e) {
					break;
				}
			}
			
			//If this thread is interrupted, set throwing to false and 
			//return (exit) the thread
			if(Thread.currentThread().isInterrupted())
			{
				throwing = false;
				return;
			}
			
			//Set the arm to 0 (no motion)
			arm.set(0);
			
			//While the arm is not at the release point
			while(!throwReleaseSwitch.get())
			{
				//Set the idler pulley to let the arm swing freely
				idlerPulley.set(DoubleSolenoid.Value.kReverse);
				
				//Try to reduce the update speed of the thread
				//so it doesn't take up all the system resources
				//If we have been interrupted (told to stop running)
				//then break out of the loop
				try {
					Thread.sleep(15);
				} catch (InterruptedException e) {
					break;
				}
			}
			
			//Regardless of whether or not we finished throwing the cube
			//or if we were interrupted, set the idler pulley back into place
			idlerPulley.set(DoubleSolenoid.Value.kForward);
			
			//Indicate that we are done throwing so we can throw again
			throwing = false;	
		}
		
	}
}
