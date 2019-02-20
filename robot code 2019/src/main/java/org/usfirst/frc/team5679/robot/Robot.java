package org.usfirst.frc.team5679.robot;
import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
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
	
	//TODO: add digital inputs for limit switches
	Joystick driveJoystick = new Joystick(0);
	Joystick functionJoystick = new Joystick(1);
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor);
	DifferentialDrive drive = new DifferentialDrive(m_left, m_right);
	
	SpeedControllerGroup scissorLift = new SpeedControllerGroup(intakeTalon);

	Encoder hatchEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	Encoder shooterEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	SendableChooser<Character> autoChooser = new SendableChooser<Character>();
	
	CameraServer camera;
	int session;	
	Instant starts;
	
	static final double wheelCircumference = 1.43;
	static final double hatchEncoderPulses = 280;
	static final double shooterEncoderPulses = 497;
	static final double shooterAngle = shooterEncoderPulses / 0.125;
	static final double clawDistancePerPulse = 360 / 7*71;
	static final double speedFactor = -1;
	static final double driveOffset = .98;

	static final double halfSpeed = .5;
	static final double minJoystickValue = 0.2;
	static final double minimumSpeed = 0.3;
	static final double slowCargoSpeed = 0.4;
	static final double fullCargoSpeed = 1;
	static final double fullSpeed = 1;
	static final double motorExpiration = .2;
	static final double autonomousSpeed = .6;
	static final double autonomousDistance = 5;
	static final double autonomousMultiplier = .95;
	static final double retrogradeSpeed = -.2;
	double speedAdjust = .7;
	double slowSpeedAdjust = .4;
	double reverseDirection = -1;

	private double slowDriveSpeed = .3;
	private double fullDriveSpeed = 1;

	static final String[] piAddresses = new String[]{
		"mjpeg:http://frcvision.local:1181/stream.mjpg",
		"mjpeg:http://frcvision.local:1182/stream.mjpg"
	};

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		starts = Instant.now();
		
		leftMotor.setExpiration(motorExpiration);
		rightMotor.setExpiration(motorExpiration);
		intakeTalon.setExpiration(motorExpiration);


		SmartDashboard.putString("robot init", "robot init");

		hatchEncoder.reset();
		shooterEncoder.reset();
		
	}	

	public void debug() {
		SmartDashboard.putNumber("Drive Joystick x", driveJoystick.getX());
		SmartDashboard.putNumber("Drive Joystick y", driveJoystick.getY());
		SmartDashboard.putNumber("FunctionJoystick x", functionJoystick.getX());
		SmartDashboard.putNumber("Function Joystick y", functionJoystick.getY());
		SmartDashboard.putNumber("Right Encoder", hatchEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", shooterEncoder.getDistance());
		SmartDashboard.putBoolean("Autonomous Right Distance Triggered", hatchEncoder.getDistance() >= autonomousDistance);
		SmartDashboard.putBoolean("Autonomous Left Distance Triggered", shooterEncoder.getDistance() >= autonomousDistance);
	}
	
	public void disabledPeriodic() {}	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		var instance = NetworkTableInstance.getDefault();
		var vision = instance.getTable("ChickenVision");
		var tapeDetected = vision.getEntry("tapeDetected");
		var cargoDetected = vision.getEntry("cargoDetected");
		var tapeYaw = vision.getEntry("tapeYaw");
		var cargoYaw = vision.getEntry("cargoYaw");
		var videoTimestamp = vision.getEntry("VideoTimestamp");
		var tapeWanted = vision.getEntry("Tape");
		var cargoWanted = vision.getEntry("Cargo");

		tapeWanted.setBoolean(false);
		cargoWanted.setBoolean(true);
		
		SmartDashboard.putBoolean("Cargo Detected", cargoDetected.getBoolean(false));
		SmartDashboard.putBoolean("Tape Detected", tapeDetected.getBoolean(false));
		SmartDashboard.putNumber("tapeYaw", tapeYaw.getDouble(0));
		SmartDashboard.putNumber("cargoYaw", cargoYaw.getDouble(0));
		SmartDashboard.putNumber("videoTimestamp", videoTimestamp.getDouble(0));

		SmartDashboard.putString("Autonomous", "Teleop");
		hatchEncoder.reset();
		shooterEncoder.reset();
		double LP = driveJoystick.getRawAxis(LEFT_AXIS);
		double RP = driveJoystick.getRawAxis(RIGHT_AXIS);
		SmartDashboard.putNumber("Left joystick", LP);
		SmartDashboard.putNumber("Right joystick", RP);
		double speedScale = speedAdjust;

		
		if (driveJoystick.getRawAxis(LEFT_TRIGGER_ID) > 0) {
			speedScale = slowDriveSpeed;
			SmartDashboard.putString("Left Trigger", "Pressed");
		}
		else if (driveJoystick.getRawAxis(RIGHT_TRIGGER_ID) > 0) {
			speedScale = fullDriveSpeed;
			SmartDashboard.putString("Right Trigger", "Pressed");
		} else {
			speedScale = speedAdjust;
			SmartDashboard.putString("Left Trigger", "Not Pressed");
		}
		//TODO: button stuff not working or however you wanna word it
		if (driveJoystick.getRawButton(A_BUTTON_ID)) { 
			if (tapeWanted.getBoolean(false)==false){
				tapeWanted.setBoolean(true);
				cargoWanted.setBoolean(false);
			}
			else {
				tapeWanted.setBoolean(false);
				cargoWanted.setBoolean(true);
			}
		}
		
		if (functionJoystick.getRawButton(RIGHT_BUMPER_ID)) {
			if (functionJoystick.getRawButton(A_BUTTON_ID)) {
				moveIntake(slowCargoSpeed * reverseDirection); 
			}
			else if (functionJoystick.getRawButton(X_BUTTON_ID)) {
				moveIntake(fullCargoSpeed * reverseDirection); 
			} else {
				moveIntake(minimumSpeed * reverseDirection);
			}	
			SmartDashboard.putString("Right Bumper", "Pressed");
		} else if (functionJoystick.getRawAxis(RIGHT_TRIGGER_ID) > 0 )  {
			if (functionJoystick.getRawButton(X_BUTTON_ID)) {
				moveIntake(slowCargoSpeed); 
			} else {
				moveIntake(minimumSpeed);
			}	
			SmartDashboard.putString("Right Trigger", "Pressed");
		} else {
			moveIntake(0);
		}

		double hatchAngle = functionJoystick.getRawAxis(LEFT_AXIS);
		if (functionJoystick.getRawAxis(LEFT_TRIGGER_ID) > 0) {

			moveHatchActuator(1);
		}

		
		if (Math.abs(RP) < minimumSpeed) {
			RP = 0;

			if (Math.abs(LP) < minimumSpeed) {
				LP = 0;
			}
		}

		setRobotDriveSpeed(LP * speedScale, RP * speedScale);

		


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
		while(hatchEncoder.get()<hatchEncoderPulses);
		hatchActuator.set(0);
		hatchEncoder.reset();
	}

	public void moveIntake (double speed){
		intakeTalon.set(speed);
	}

	
}