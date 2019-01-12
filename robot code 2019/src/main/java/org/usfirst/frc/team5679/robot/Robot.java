package org.usfirst.frc.team5679.robot;
import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
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
	private static final double CLAW_OPEN_CLOSE_SPEED = 0.6;
	private static final double SCISSOR_LIFT_SPEED = .7;
	private static final double CLAW_RAISE_LOWER_SPEED = 0.6;
	private static final Duration AUTONOMOUS_CLAW_SECONDS = Duration.ofSeconds(2);
	
	Talon leftMotor0 = new Talon(0);
	Talon leftMotor1 = new Talon(1);
	Talon rightMotor0 = new Talon(2);
	Talon rightMotor1 = new Talon(3);
	Talon leftScissorLiftActuator = new Talon(4);
	Talon rightScissorLiftActuator = new Talon(5);
	Victor tiltClawActuator = new Victor(6);
	Talon clawActuator = new Talon(7);
	
	//DigitalInput limitSwitchLiftTop = new DigitalInput(6);
	DigitalInput limitSwitchClawLower = new DigitalInput(7);
	DigitalInput limitSwitchClawOpen = new DigitalInput(4);
	DigitalInput limitSwitchClawClose = new DigitalInput(6);
	
	Joystick driveJoystick = new Joystick(0);
	Joystick clawJoystick = new Joystick(1);
	SpeedControllerGroup m_left = new SpeedControllerGroup(leftMotor0, leftMotor1);
	SpeedControllerGroup m_right = new SpeedControllerGroup(rightMotor0, rightMotor1);
	DifferentialDrive drive = new DifferentialDrive(m_left, m_right);
	
	SpeedControllerGroup scissorLift = new SpeedControllerGroup(leftScissorLiftActuator, rightScissorLiftActuator);

	Encoder rightEncoder = new Encoder(2, 3, false, EncodingType.k4X);
	Encoder leftEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	SendableChooser<Character> autoChooser = new SendableChooser<Character>();
	
	CameraServer camera;
	int session;	
	String gameData;
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
		CameraServer.getInstance().startAutomaticCapture();
		starts = Instant.now();
		gameData = DriverStation.getInstance().getGameSpecificMessage().trim();
		
		if (!gameData.isEmpty()) {
			homeSwitchDirection = gameData.charAt(0);
			middleScaleDirection = gameData.charAt(1);
			opponentSwitchDirection = gameData.charAt(2);
		}
		
		SmartDashboard.putString("gameData", gameData);
		SmartDashboard.putString("homeSwitchDirection", homeSwitchDirection + "");
		SmartDashboard.putString("middleScaleDirection", middleScaleDirection + "");
		SmartDashboard.putString("opponentSwitchDirection", opponentSwitchDirection + "");
		
		leftMotor0.setExpiration(motorExpiration);
		leftMotor1.setExpiration(motorExpiration);
		rightMotor0.setExpiration(motorExpiration);
		rightMotor1.setExpiration(motorExpiration);
		leftScissorLiftActuator.setExpiration(motorExpiration);
		tiltClawActuator.setExpiration(motorExpiration);
		clawActuator.setExpiration(motorExpiration);

		rightEncoder.setDistancePerPulse(distancePerPulse);
		leftEncoder.setDistancePerPulse(distancePerPulse);

		SmartDashboard.putString("robot init", "robot init");

		rightEncoder.reset();
		leftEncoder.reset();	
		autoChooser.setDefaultOption("left", 'L');
		autoChooser.setDefaultOption("Center", 'C');
		autoChooser.setDefaultOption("Right", 'R');
		SmartDashboard.putData("Direction Chooser", autoChooser);
	}	

	/**
	 * This function sets up any necessary data before the autonomous control loop.
	 */
	public void autonomousinit() {
		SmartDashboard.putString("Autonomous", "Init");
		gameData = DriverStation.getInstance().getGameSpecificMessage().trim();
		starts = null; 
		if (!gameData.isEmpty()) {
			homeSwitchDirection = gameData.charAt(0);
			middleScaleDirection = gameData.charAt(1);
			opponentSwitchDirection = gameData.charAt(2);
		}
		
		SmartDashboard.putString("gameData", gameData);
		
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
        		Instant ends = Instant.now();
        		SmartDashboard.putNumber("Autonomous end timer", starts.getNano() / 1000);
        		Duration clawDuration = Duration.between(starts, ends);        			
        		if (clawDuration.compareTo(AUTONOMOUS_CLAW_SECONDS) <= 0) {
            		lowerClaw(CLAW_RAISE_LOWER_SPEED);
            	}
            	else {
            		tiltClawActuator.set(0);
            		if (limitSwitchClawOpen.get()) {
                		openClaw(CLAW_OPEN_CLOSE_SPEED);            			
            		} 
            	}       
            }
		}
		else {
			setRobotDriveSpeed(autonomousSpeed, autonomousSpeed * autonomousMultiplier);
			SmartDashboard.putString("Autonomous", "Go");
			starts = Instant.now();
		}
		
		SmartDashboard.putNumber("Autonomous start timer", starts.getNano() / 1000);
		
	}
	
	public void disabledPeriodic() {
		gameData = DriverStation.getInstance().getGameSpecificMessage().trim();
		
		if (!gameData.isEmpty()) {
			homeSwitchDirection = gameData.charAt(0);
			middleScaleDirection = gameData.charAt(1);
			opponentSwitchDirection = gameData.charAt(2);
		}
		
		SmartDashboard.putString("gameData", gameData);
	}
	
	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("Autonomous", "Teleop");
		SmartDashboard.putBoolean("limit lift bottom", limitSwitchClawLower.get());
		SmartDashboard.putBoolean("limit lift claw open", limitSwitchClawOpen.get());
		SmartDashboard.putBoolean("limit lift claw close", limitSwitchClawClose.get());
		rightEncoder.reset();
		leftEncoder.reset();
		double LP = driveJoystick.getRawAxis(LEFT_AXIS);
		double RP = driveJoystick.getRawAxis(RIGHT_AXIS);

		if (clawJoystick.getRawButton(LEFT_BUMPER_ID)) {
			lowerClaw(CLAW_RAISE_LOWER_SPEED);
			SmartDashboard.putString("Right Trigger", "Pressed");
		} else if (clawJoystick.getRawButton(RIGHT_BUMPER_ID)) {
			raiseClaw(CLAW_RAISE_LOWER_SPEED);
			SmartDashboard.putString("Right Bumper", "Pressed");
		} else {
			SmartDashboard.putString("Right Trigger", "Not Pressed");
			tiltClawActuator.set(0);
		}
		
		if (driveJoystick.getRawAxis(LEFT_TRIGGER_ID) > 0) {
			SmartDashboard.putString("Left Trigger", "Pressed");
			// Send negative scissor lift speed to lower scissor lift
			if (limitSwitchClawLower.get()) {
				moveScissorLift(SCISSOR_LIFT_SPEED * -1);
			}
		} else {
			SmartDashboard.putString("Left Trigger", "Not Pressed");
		}
		
		if (driveJoystick.getRawButton(LEFT_BUMPER_ID)) {
			SmartDashboard.putString("Left Bumper", "Pressed");
			//if (limitSwitchLiftTop.get()) {
				moveScissorLift(SCISSOR_LIFT_SPEED * 1);
			//}
		} else {	
			SmartDashboard.putString("Left Bumper", "Not Pressed");
		}

		if (clawJoystick.getRawButton(A_BUTTON_ID) && limitSwitchClawOpen.get()) {
			SmartDashboard.putString("A BUTTON", "Pressed");
			SmartDashboard.putNumber("Open Speed", CLAW_OPEN_CLOSE_SPEED);
			openClaw(CLAW_OPEN_CLOSE_SPEED);
		} 		
		else if (clawJoystick.getRawButton(B_BUTTON_ID) && limitSwitchClawClose.get()) {
			SmartDashboard.putString("B BUTTON", "Pressed");
			closeClaw(CLAW_OPEN_CLOSE_SPEED);
		} else {
			SmartDashboard.putString("B BUTTON", "Not Pressed");
			clawActuator.set(0);
			SmartDashboard.putString("A BUTTON", "Not Pressed");
			SmartDashboard.putNumber("Open Speed", 0);
		}
		
		if (Math.abs(RP) < minimumSpeed) {
			RP = 0;

			if (Math.abs(LP) < minimumSpeed) {
				LP = 0;
			}
		}

		setRobotDriveSpeed(-RP * speedAdjust, -LP * speedAdjust);

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

	public void moveScissorLift(double speed) {
		scissorLift.set(speed);
	}

	public void openClaw(double speed) {
		clawActuator.set(speed);
	}

	public void closeClaw(double speed) {
		clawActuator.set(speed * -1);
	}

	public void raiseClaw(double speed) {
		tiltClawActuator.set(speed);
	}

	public void lowerClaw(double speed) {
		tiltClawActuator.set(speed * -1);
	}
}