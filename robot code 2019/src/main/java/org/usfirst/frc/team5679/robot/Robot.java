package org.usfirst.frc.team5679.robot;
import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
//import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
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

/**
 * @author Robotics
 *
 */
public class Robot extends TimedRobot {
	private static final int RIGHT_AXIS = 5;
	private static final int LEFT_AXIS = 1;
	private static final int B_BUTTON_ID = 2;
	private static final int A_BUTTON_ID = 1;
	private static final int LEFT_BUMPER_ID = 5;
	private static final int RIGHT_BUMPER_ID = 6;
	private static final int LEFT_TRIGGER_ID = 2;
	private static final int RIGHT_TRIGGER_ID = 3;
	private static final int X_BUTTON_ID = 3;
	private static final int Y_BUTTON_ID = 4;
	
	//TODO: make sure we have a definite number of talons and their ports (0, 1, 2, ...).
	Talon leftMotor = new Talon(0);
	Talon rightMotor = new Talon(1);
	Talon intakeTalon = new Talon(4);
	Talon hatchActuator = new Talon(6);
	//DoubleSolenoid ballReturn = new DoubleSolenoid(7, 8);
	
	Joystick driveJoystick = new Joystick(0);
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor);
	DifferentialDrive drive = new DifferentialDrive(m_left, m_right);
	
	SpeedControllerGroup scissorLift = new SpeedControllerGroup(intakeTalon);

	Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	SendableChooser<Character> autoChooser = new SendableChooser<Character>();
	
	CameraServer camera;
	int session;	
	Instant starts;
	
	static final double wheelCircumference = 1.43;
	static final double encoderPulses = 250;
	static final double distancePerPulse = wheelCircumference / encoderPulses;
	static final double clawDistancePerPulse = 360 / 7*71;
	static final double speedFactor = -1;
	static final double driveOffset = .98;

	static final double halfSpeed = .5;
	static final double minJoystickValue = 0.2;
	static final double minimumSpeed = 0.1;
	static final int fullSpeed = 1;
	static final double motorExpiration = .2;
	static final double autonomousSpeed = .6;
	static final double autonomousDistance = 5;
	static final double autonomousMultiplier = .95;
	static final double retrogradeSpeed = -.2;
	double speedAdjust = .8;

	// Defining which panels are exactly where on the field.
	char homeSwitchDirection;
	char opponentSwitchDirection;
	char middleScaleDirection;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		SmartDashboard.putString("Autonomous", "Robot Init");
		//CameraServer.getInstance().startAutomaticCapture();
		starts = Instant.now();
		
		leftMotor.setExpiration(motorExpiration);
		rightMotor.setExpiration(motorExpiration);
		intakeTalon.setExpiration(motorExpiration);

		rightEncoder.setDistancePerPulse(distancePerPulse);
		leftEncoder.setDistancePerPulse(distancePerPulse);

		SmartDashboard.putString("robot init", "robot init");

		rightEncoder.reset();
		leftEncoder.reset();	
	}	

	/**
	 * This function sets up any necessary data before the autonomous control loop.
	 */
	public void autonomousinit() {
		SmartDashboard.putString("Autonomous", "Init");
		starts = null; 
				
		rightEncoder.reset();
		leftEncoder.reset();
	}

	public void debug() {
		SmartDashboard.putNumber("Joystick x", driveJoystick.getX());
		SmartDashboard.putNumber("Joystick y", driveJoystick.getY());
		SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", leftEncoder.getDistance());
		SmartDashboard.putBoolean("Autonomous Right Distance Triggered", rightEncoder.getDistance() >= autonomousDistance);
		SmartDashboard.putBoolean("Autonomous Left Distance Triggered", leftEncoder.getDistance() >= autonomousDistance);
	}

	/**
	 * This function is called periodically during autonomous control
	 */
	@Override
	public void autonomousPeriodic() {
		debug();
		
		if (Math.abs(rightEncoder.getDistance()) >= autonomousDistance || 
			Math.abs(leftEncoder.getDistance()) >= autonomousDistance) {
			SmartDashboard.putString("Autonomous", "Stop");
            drive.tankDrive(retrogradeSpeed, retrogradeSpeed);
            drive.tankDrive(0, 0);
            if (homeSwitchDirection == autoChooser.getSelected()) {
        		SmartDashboard.putNumber("Autonomous end timer", starts.getNano() / 1000);		       
            }
		}
		else {
			setRobotDriveSpeed(autonomousSpeed, autonomousSpeed * autonomousMultiplier);
			SmartDashboard.putString("Autonomous", "Go");
			starts = Instant.now();
		}
		
		SmartDashboard.putNumber("Autonomous start timer", starts.getNano() / 1000);
		
	}
	
	public void disabledPeriodic() {}	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("Autonomous", "Teleop");
		rightEncoder.reset();
		leftEncoder.reset();
		double LP = driveJoystick.getRawAxis(LEFT_AXIS);
		double RP = driveJoystick.getRawAxis(RIGHT_AXIS);
		SmartDashboard.putNumber("Left joystick", LP);
		SmartDashboard.putNumber("Right joystick", RP);

		
		if (driveJoystick.getRawAxis(LEFT_TRIGGER_ID) > 0) {
			SmartDashboard.putString("Left Trigger", "Pressed");
		} else {
			SmartDashboard.putString("Left Trigger", "Not Pressed");
		}
		
		if (driveJoystick.getRawButton(LEFT_BUMPER_ID)) {
			moveIntake(1);
			SmartDashboard.putString("Left Bumper", "Pressed");
		} else if (driveJoystick.getRawButton(RIGHT_BUMPER_ID)) {
			moveIntake(-.85);	
			SmartDashboard.putString("Right Bumper", "Pressed");
		} else {
			moveIntake(0);
		}

		if (driveJoystick.getRawButton(A_BUTTON_ID)) {
			moveHatchActuator(1);
		}
		else if (driveJoystick.getRawButton(B_BUTTON_ID)){
			moveHatchActuator(-1);
		}
		else {
			moveHatchActuator(0);
		}

		
		if (Math.abs(RP) < minimumSpeed) {
			RP = 0;

			if (Math.abs(LP) < minimumSpeed) {
				LP = 0;
			}
		}

		setRobotDriveSpeed(LP * speedAdjust, RP * speedAdjust);

	}

	/**
	 * This method sets the speed and applies the limiting speed factor for
	 * SpeedControllers (motor)
	 * 
	 * @param motor
	 *            the motor for which we are setting the speed.
	 * @param speed
	 *            to which we are setting the motor (base speed)
	 */
	public void setMotorSpeed(SpeedController motor, double speed) {
		motor.set(speed * speedFactor);
	}

	/**
	 * This method sets the speed and applies the limiting speed factor for robot
	 * Tank Drive
	 * 
	 * @param leftSpeed
	 * @param rightSpeed
	 */
	public void setRobotDriveSpeed(double leftSpeed, double rightSpeed) {
		SmartDashboard.putNumber("leftspeed", leftSpeed);
		SmartDashboard.putNumber("rightspeed", rightSpeed);

		drive.tankDrive(leftSpeed * speedFactor, rightSpeed * speedFactor);
	}

	public void moveHatchActuator (double speed) {
		hatchActuator.set(speed);
	}

	public void moveIntake (double speed){
		intakeTalon.set(speed);
	}

	
}