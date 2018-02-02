package org.usfirst.frc.team2658.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */

public class Robot extends IterativeRobot {
	/* Author --> Gokul Swaminathan */
	final int FRONT_LEFT_PORT = 0;   				//port number for front left motor
	final int BACK_LEFT_PORT = 1;					//port number for back left motor
	final int FRONT_RIGHT_PORT = 3;					//port number for front right motor
	final int BACK_RIGHT_PORT = 4;					//port number for back right motor
	final int XBOX_PORT = 0;						//xbox remote port
	final int ANDREWS_JOY1 = 1;						//joystick 1 port
	final int ANDREWS_JOY2 = 2;						//joystick 2 port
	
	final int CHOOSE_XBOX = 0, CHOOSE_DUAL = 1;		//chooser id's
	
	//motors
	Talon fLeft = new Talon(FRONT_LEFT_PORT);
	Talon fRight = new Talon(FRONT_RIGHT_PORT);
	Talon bLeft = new Talon(BACK_LEFT_PORT);
	Talon bRight = new Talon(BACK_RIGHT_PORT);
	
	//groups of motors
	SpeedControllerGroup spLeft = new SpeedControllerGroup(fLeft, bLeft); 
	SpeedControllerGroup spRight = new SpeedControllerGroup(fRight, bRight);

	//drivetrain
	DifferentialDrive driveTrain = new DifferentialDrive(spLeft, spRight);
	
	//controllers
	Joystick xbox = new Joystick(XBOX_PORT);
	Joystick joyLeft = new Joystick(ANDREWS_JOY1);
	Joystick joyRight = new Joystick(ANDREWS_JOY2);
	
	//strings for chooser
	String power = "Drive Power";
	String sensitivity = "Sensitivity";
	/* Author --> Gokul Swaminathan */
	
	SendableChooser<Integer> chooser = new SendableChooser<>();
	
	
	// Autonomous Variables
	DriverStation driverStation;
	String fmsMessage;
	
	Encoder rEncoder, lEncoder;
	int encoderAvg;
	char switchSide, scaleSide;
	double encDist, distanceTraveled;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		/* Author --> Gokul Swaminathan */
		chooser.addDefault("Xbox Controller (Tank Drive)", CHOOSE_XBOX);
		chooser.addObject("Dual Joysticks", CHOOSE_DUAL);
		SmartDashboard.putData("Drive choices", chooser);
		SmartDashboard.putNumber(power, 1);
		SmartDashboard.putNumber(sensitivity, 2);
		/* Author --> Gokul Swaminathan */
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		//init vars
		fmsMessage = driverStation.getGameSpecificMessage();
		switchSide = fmsMessage.charAt(0);
		scaleSide = fmsMessage.charAt(1);
		distanceTraveled = 0;
		encDist = Math.PI * 6;
		
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		encoderAvg = (rEncoder.get() + lEncoder.get())/2;
		distanceTraveled = encDist*encoderAvg;
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		
		/* Author --> Gokul Swaminathan */
		//get inputs from user
		double exponent = SmartDashboard.getNumber(sensitivity, 2);
		double constant = SmartDashboard.getNumber(power, 1);
		
		final int xboxRightAxis = 5;
		final int xboxLeftAxis = 1;
		
		final int joyRightAxis = 1;
		final int joyLeftAxis = 1;
		
		int mode = chooser.getSelected();
		
		switch(mode)
		{
		case CHOOSE_XBOX: 
			//run tank drive method for xbox
			andrewTankDrive(xbox, xbox, exponent, constant, driveTrain, xboxLeftAxis, xboxRightAxis);
			break;
			
		case CHOOSE_DUAL:
			//run tank drive method for joysticks
			andrewTankDrive(joyRight, joyLeft, exponent, constant, driveTrain, joyLeftAxis, joyRightAxis);
			break;
			
		default:
			break;
		}
		/* Author --> Gokul Swaminathan */
		
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}
	
	/* Author --> Gokul Swaminathan */
	public void andrewTankDrive(Joystick inputR, Joystick inputL,  double exp, double cons, DifferentialDrive drive, int leftStick, int rightStick )
	{
		int negR = 0, negL = 0;
		
		if(inputR.getRawAxis(rightStick) < 0)
		{
			negR = -1;			
		}
		else if(inputR.getRawAxis(rightStick) > 0)
		{
			negR = 1;
		}
		if(inputL.getRawAxis(leftStick) < 0)
		{
			negL = -1;
		}
		else if(inputL.getRawAxis(leftStick) > 0)
		{
			negL = 1;
		}
		
		
		
		double leftSpeed = - negR * cons * Math.pow(Math.abs(inputR.getRawAxis(5)), exp);
		double rightSpeed = - negL *  cons * Math.pow(Math.abs(inputL.getRawAxis(1)), exp);
		
		drive.tankDrive(leftSpeed, rightSpeed);
	}
	/* Author --> Gokul Swaminathan */
}

