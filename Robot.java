/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3175.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with arcade drive.
 */
public class Robot extends IterativeRobot {

	// scissor lift preset heights
	private static final double SWITCH = -10.5 * 4096;
	private static final double SCALE = -35.0 * 4096;
	private static final double CLIMB = -40.0 * 4096;
	private static final double MAX_HEIGHT = -41.0 * 4096;

	/** SET THIS BEFORE MATCH! **/
	private static final String GOAL = "scale";
	private static final int LOCATION = 1;
	private static final String DRIVE_MODE = "tank";

	// arcade drive speeds
	private static int gear = 1;

	private Timer runTime = new Timer();

	// Joystick
	private XboxController driver;
	private Joystick tankDriver;
	private XboxController operator;

	// Drive train
	private Victor leftDrive;
	private Victor rightDrive;
	private DifferentialDrive driveTrain;

	// lift system
	private Victor winch;
	private TalonSRX leftScissor;

	private Victor intake;
	private Victor intakeLift;
	private Victor deploy;

	// pneumatics
	private DoubleSolenoid intakeArm;
	private Compressor compressor;

	// Alliance color
	private static Alliance alliance;
	// alliance station number of the robot in auton
	private static int station;
	// locations of the alliance switch and scale
	public String field;

	/**
	 * This function is run once each time the robot turns on.
	 */
	@Override
	public void robotInit() {
		SmartDashboard.putString("Robot Init", "Initializing...");

		if (DRIVE_MODE.equals("arcade")) {
			driver = new XboxController(0);
		} else {
			tankDriver = new Joystick(0);
		}
		operator = new XboxController(1);

		leftDrive = new Victor(0);
		rightDrive = new Victor(1);
		driveTrain = new DifferentialDrive(leftDrive, rightDrive);

		intake = new Victor(3);
		intakeLift = new Victor(5);
		deploy = new Victor(4);

		winch = new Victor(2);
		leftScissor = new TalonSRX(5);
		leftScissor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative,
				0, 10);
		leftScissor.setSensorPhase(true);
		/* set closed loop gains in slot0, typically kF stays zero. */
		leftScissor.config_kF(0, 0.0, 10);
		leftScissor.config_kP(0, 0.1, 10);
		leftScissor.config_kI(0, 0.0, 10);
		leftScissor.config_kD(0, 0.0, 10);
		/* set the peak and nominal outputs, 12V means full */
		// leftScissor.configNominalOutputForward(0, 10);
		// leftScissor.configNominalOutputReverse(0, 10);
		// leftScissor.configPeakOutputForward(2, 10);
		// leftScissor.configPeakOutputReverse(-1, 10);

		intakeArm = new DoubleSolenoid(4, 5);
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);

		// CameraServer.getInstance().startAutomaticCapture();

		SmartDashboard.putString("Robot Init", "Initialized!");
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		SmartDashboard.putString("Auton Init", "Initializing...");

		leftScissor.setSelectedSensorPosition(0, 0, 10);
		field = DriverStation.getInstance().getGameSpecificMessage();
		alliance = DriverStation.getInstance().getAlliance();
		station = DriverStation.getInstance().getLocation();

		runTime.reset();
		runTime.start();

		SmartDashboard.putString("Alliance", alliance + Integer.toString(station));
		SmartDashboard.putString("Position", alliance + Integer.toString(LOCATION));
		SmartDashboard.putString("Auton Init", "Initialized!");
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// flip down the intake arms
		if (runTime.get() < 0.2) {
			deploy.set(-0.5);
		} else if (runTime.get() < 0.5) {
			deploy.set(0);
		}
		if (!GOAL.equals("cross") && runTime.get() > 0.5) {
			switch (LOCATION) {
			case 1: // alliance station 1 (left)
				if (field.charAt(0) == 'L' && GOAL.equals("switch")) {
					// go straight to the switch and put the cube in
					straightSwitch();
				} else if (field.charAt(0) == 'R' && GOAL.equals("switch")) {
					// switch on the right Goes the long way around the switch (avoid collisions)
					rightSwitch1();
				} else if (field.charAt(1) == 'L' && GOAL.equals("scale")) {
					// scale on the left drive forward lift and drop at the scale
					// go straight to the scale and put the cube in
					leftScale1();
				} else if (field.charAt(1) == 'R' && GOAL.equals("scale")) {
					// scale on the right drive forward turn right
					rightScale1();
				}
				break;
			case 2: // alliance station 2 (middle)
				if (field.charAt(0) == 'L' && GOAL.equals("switch")) {
					// switch on the left turns left and put the block
					leftSwitch2();
				} else if (field.charAt(0) == 'R' && GOAL.equals("switch")) {
					// switch on the right turns right and put the block
					rightSwitch2();
				} else if (field.charAt(1) == 'L' && GOAL.equals("scale")) {
					// scale on the left turn left and go the long way around the switch
					leftScale2();
				} else if (field.charAt(1) == 'R' && GOAL.equals("scale")) {
					// scale on the right turn right and go the long way around the switch
					rightScale2();
				}
				break;
			case 3: // alliance station 3 (right)
				if (field.charAt(0) == 'L' && GOAL.equals("switch")) {
					// switch on the left Goes the long way around the switch (avoid collisions)
					leftSwitch3();
				} else if (field.charAt(0) == 'R' && GOAL.equals("switch")) {
					// switch on the right Goes straight to the switch and puts cube in
					straightSwitch();
				} else if (field.charAt(1) == 'L' && GOAL.equals("scale")) {
					// scale on the left go forward turn left
					leftScale3();
				} else if (field.charAt(1) == 'R' && GOAL.equals("scale")) {
					// scale on the right go forward to the scale
					rightScale3();
				}
				break;
			}
		} else if (GOAL.equals("cross") && runTime.get() > 0.5) {
			// cross the auton line
			if (runTime.get() < 1.8) {
				driveTrain.arcadeDrive(0.75, 0);
			}
		}
	}

	/**
	 * This function is run once each time the robot enters teleop mode.
	 */
	@Override
	public void teleopInit() {
		SmartDashboard.putString("Teleop Init", "Initializing...");

		leftScissor.setSelectedSensorPosition(0, 0, 10);
		leftScissor.setNeutralMode(NeutralMode.Brake);
		// Configure Talon to clear sensor position on Forward Limit
		leftScissor.configSetParameter(ParamEnum.eClearPositionOnLimitF, 1, 0, 0, 10);

		SmartDashboard.putString("Teleop Init", "Initialized!");
	}

	/**
	 * This function is called periodically during teleop.
	 */
	@Override
	public void teleopPeriodic() {
		telemetry();
		drive();
		intakeControl();
		armControl();
		winchControl();
		scissorControl();
		deployArm();
		intakeLiftControl();
	}

	/*
	 * auton placing block on switch when on same side. left switch for alliance
	 * station 1, or right switch for alliance station 3
	 */
	private void straightSwitch() {
		if (runTime.get() < 1.8) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 1.3 seconds
		} else if (runTime.get() < 4.8) {
			leftScissor.set(ControlMode.Position, SWITCH); // scissor lift raises to switch level
		} else if (runTime.get() < 5.0) {
			intakeArm.set(DoubleSolenoid.Value.kForward); // release block
		}
	}

	/*
	 * auton switch for the right switch on alliance station 1 (opposite side)
	 */
	private void rightSwitch1() {
		if (runTime.get() < 2.6) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 2.1 seconds
		} else if (runTime.get() < 2.9) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 4.9) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 2 seconds
		} else if (runTime.get() < 5.2) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 5.5) {
			driveTrain.arcadeDrive(0.5, 0); // Goes forward for 0.3 seconds
		} else if (runTime.get() < 5.8) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 8.8) {
			leftScissor.set(ControlMode.Position, SWITCH);
		} else if (runTime.get() < 9.0) {
			intakeArm.set(DoubleSolenoid.Value.kForward);
		}
	}

	/*
	 * auton for left scale on alliance station 1 (same side)
	 */
	private void leftScale1() {
		if (runTime.get() < 2.6) {
			driveTrain.arcadeDrive(1, 0); // Goes forward for 2.1 seconds at full speed
		} else if (runTime.get() < 2.9) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 9.4) {
			leftScissor.set(ControlMode.Position, SWITCH); // scissor lift raises to switch level
		} else if (runTime.get() < 9.6) {
			intakeArm.set(DoubleSolenoid.Value.kForward); // release block
		}
	}

	/*
	 * auton for right scale on alliance station 1 (Opposite side)
	 */
	private void rightScale1() {
		if (runTime.get() < 2.6) {
			driveTrain.arcadeDrive(1, 0); // Goes forward for 2.1 seconds
		} else if (runTime.get() < 2.9) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 4.9) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 2 seconds
		} else if (runTime.get() < 5.2) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 11.7) {
			leftScissor.set(ControlMode.Position, SWITCH);
		} else if (runTime.get() < 11.9) {
			intakeArm.set(DoubleSolenoid.Value.kForward);
		}
	}

	/*
	 * auton for left switch alliance station 2 (middle left)
	 */
	private void leftSwitch2() {
		if (runTime.get() < 1.5) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 1 second
		} else if (runTime.get() < 1.8) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 2.3) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 0.5 second
		} else if (runTime.get() < 2.6) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 3.2) {
			driveTrain.arcadeDrive(0.5, 0); // Goes forward for 0.6 seconds
		} else if (runTime.get() < 6.2) {
			leftScissor.set(ControlMode.Position, SWITCH); // scissor lift raises to switch level
		} else if (runTime.get() < 6.5) {
			intakeArm.set(DoubleSolenoid.Value.kForward); // release block
		}
	}

	/*
	 * auton for the right switch on alliance station 2 (middle right)
	 */
	private void rightSwitch2() {
		if (runTime.get() < 1.5) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 1 second
		} else if (runTime.get() < 1.8) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 2.3) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 0.5 second
		} else if (runTime.get() < 2.6) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 3.2) {
			driveTrain.arcadeDrive(0.5, 0); // Goes forward for 0.6 seconds
		} else if (runTime.get() < 6.2) {
			leftScissor.set(ControlMode.Position, SWITCH); // scissor lift raises to switch level
		} else if (runTime.get() < 6.4) {
			intakeArm.set(DoubleSolenoid.Value.kForward); // release block
		}
	}

	/*
	 * auton for left scale on alliance station 2
	 */
	private void leftScale2() {
		if (runTime.get() < 1.5) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 1 second
		} else if (runTime.get() < 1.8) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 2.5) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 0.7 second
		} else if (runTime.get() < 2.9) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 4.0) {
			driveTrain.arcadeDrive(1, 0); // Goes forward for 1.1 seconds
		} else if (runTime.get() < 10.5) {
			leftScissor.set(ControlMode.Position, SWITCH); // scissor lift raises to switch level
		} else if (runTime.get() < 10.7) {
			intakeArm.set(DoubleSolenoid.Value.kForward); // release block
		}
	}

	/*
	 * auton for right scale alliance station 2
	 */
	private void rightScale2() {
		if (runTime.get() < 1.5) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 1 second
		} else if (runTime.get() < 1.8) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 2.5) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 0.7 second
		} else if (runTime.get() < 2.9) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 4.0) {
			driveTrain.arcadeDrive(1, 0); // Goes forward for 1.1 seconds
		} else if (runTime.get() < 10.5) {
			leftScissor.set(ControlMode.Position, SWITCH); // scissor lift raises to switch level
		} else if (runTime.get() < 10.7) {
			intakeArm.set(DoubleSolenoid.Value.kForward); // release block
		}
	}

	/*
	 * auton for left switch alliance station 3 (opposite)
	 */
	private void leftSwitch3() {
		if (runTime.get() < 2.6) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 2.1 seconds
		} else if (runTime.get() < 2.9) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 4.9) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 2 seconds
		} else if (runTime.get() < 5.2) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 5.5) {
			driveTrain.arcadeDrive(0.5, 0); // Goes forward for 0.3 seconds
		} else if (runTime.get() < 5.8) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 8.8) {
			leftScissor.set(ControlMode.Position, SWITCH);
		} else if (runTime.get() < 9.0) {
			intakeArm.set(DoubleSolenoid.Value.kForward);
		}
	}

	/*
	 * auton for left scale alliance station 3 (opposite)
	 */
	private void leftScale3() {
		if (runTime.get() < 2.6) {
			driveTrain.arcadeDrive(1, 0); // Goes forward for 2.1 seconds
		} else if (runTime.get() < 2.9) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 4.9) {
			driveTrain.arcadeDrive(0.75, 0); // Goes forward for 2 seconds
		} else if (runTime.get() < 5.2) {
			// Turns right (0.3 seconds)
			leftDrive.set(0.5);
			rightDrive.set(0.5);
		} else if (runTime.get() < 11.7) {
			leftScissor.set(ControlMode.Position, SWITCH);
		} else if (runTime.get() < 11.9) {
			intakeArm.set(DoubleSolenoid.Value.kForward);
		}
	}

	/*
	 * auton for right scale alliance 3 (same side)
	 */
	private void rightScale3() {
		if (runTime.get() < 2.6) {
			driveTrain.arcadeDrive(1, 0); // Goes forward for 2.1 seconds at full speed
		} else if (runTime.get() < 2.9) {
			// Turns left (0.3 seconds)
			leftDrive.set(-0.5);
			rightDrive.set(-0.5);
		} else if (runTime.get() < 9.4) {
			leftScissor.set(ControlMode.Position, SWITCH); // scissor lift raises to switch level
		} else if (runTime.get() < 9.6) {
			intakeArm.set(DoubleSolenoid.Value.kForward); // release block
		}
	}

	/*
	 * Sends robot data to the SmartDashboard
	 */
	private void telemetry() {
		// encoder readings
		SmartDashboard.putNumber("Scissor lift position: ", leftScissor.getSelectedSensorPosition(0) / 4096.0);
		// pneumatics
		SmartDashboard.putBoolean("Compressor enabled", compressor.enabled());
		SmartDashboard.putBoolean("Pressure Switch On", compressor.getPressureSwitchValue());
		// drive train
		if (gear == 1) {
			SmartDashboard.putString("Gear", "30% Speed");
		} else if (gear == 2) {
			SmartDashboard.putString("Gear", "40% Speed");
		} else if (gear == 3) {
			SmartDashboard.putString("Gear", "60% Speed");
		} else if (gear == 4) {
			SmartDashboard.putString("Gear", "80% Speed");
		} else if (gear == 5) {
			SmartDashboard.putString("Gear", "100% Speed");
		}
		// runtime
		SmartDashboard.putString("Run time", Double.toString(runTime.get()));
	}

	/*
	 * Programmable four speed arcade drive
	 */
	private void drive() {
		if (DRIVE_MODE.equals("arcade")) {
			double turnSpeed;
			if (Math.abs(driver.getRawAxis(0)) > 0.3) {
				turnSpeed = -driver.getRawAxis(0);
			} else {
				turnSpeed = 0;
			}
			driveTrain.arcadeDrive(driver.getRawAxis(1), turnSpeed);
			gear = 5;
			// Gears of the drive train left joystick
			if (driver.getYButton()) {
				// gear 1 40% speed
				driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.4, turnSpeed * 0.4);
				gear = 2;
			} else if (driver.getBButton()) { // Listens for A button
				// While A button is held it executes the normal code at 80%
				driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.6, turnSpeed * 0.6);
				gear = 3;
			} else if (driver.getAButton()) {
				// While B button is held it executes the normal code at 60%
				driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.8, turnSpeed * 0.8);
				gear = 4;
			} else if (driver.getXButton()) {
				driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.3, turnSpeed * 0.3);
				gear = 1;
			}
		} else if (DRIVE_MODE.equals("tank")) {
			if (Math.abs(tankDriver.getRawAxis(1) - tankDriver.getRawAxis(5)) > 0.2) {
				driveTrain.tankDrive(-tankDriver.getRawAxis(1), -tankDriver.getRawAxis(5));
			} else {
				driveTrain.tankDrive(-Math.max(tankDriver.getRawAxis(1), tankDriver.getRawAxis(5)),
						-Math.max(tankDriver.getRawAxis(1), tankDriver.getRawAxis(5)));
			}
			gear = 5; // 100% speed
			if (tankDriver.getRawButton(5)) {
				// left bumper button 80% speed
				if (Math.abs(tankDriver.getRawAxis(1) - tankDriver.getRawAxis(5)) > 0.2) {
					driveTrain.tankDrive(-tankDriver.getRawAxis(1) * 0.8, -tankDriver.getRawAxis(5) * 0.8);
				} else {
					driveTrain.tankDrive(-0.8 * Math.max(tankDriver.getRawAxis(1), tankDriver.getRawAxis(5)),
							-0.8 * Math.max(tankDriver.getRawAxis(1), tankDriver.getRawAxis(5)));
				}
				gear = 4;
			} else if (tankDriver.getRawButton(6)) {
				// right bumper button 40% speed
				if (Math.abs(tankDriver.getRawAxis(1) - tankDriver.getRawAxis(5)) > 0.2) {
					driveTrain.tankDrive(-tankDriver.getRawAxis(1) * 0.4, -tankDriver.getRawAxis(5) * 0.4);
				} else {
					driveTrain.tankDrive(-0.4 * Math.max(tankDriver.getRawAxis(1), tankDriver.getRawAxis(5)),
							-0.4 * Math.max(tankDriver.getRawAxis(1), tankDriver.getRawAxis(5)));
				}
				gear = 2;
			}
		}
	}

	/*
	 * controls the wheeled intake B intake A spits out
	 */
	private void intakeControl() {
		// Operator Stick Intakes
		if (operator.getAButton()) {
			intake.set(1); // A spit out
		} else if (operator.getBButton()) {
			intake.set(-0.5); // B intake
		} else {
			intake.set(0); // stop motor
		}
	}

	/*
	 * controls the movement of the pneumatic arm intake. operator x closes up y
	 * opens
	 */
	private void armControl() {
		if (operator.getXButton()) {
			intakeArm.set(DoubleSolenoid.Value.kReverse);
		} else if (operator.getYButton()) {
			intakeArm.set(DoubleSolenoid.Value.kForward);
		}
	}

	/*
	 * controls the movement of the scissor lift.
	 */
	private void scissorControl() {
		// scissor lift right joystick y override
		if (Math.abs(operator.getRawAxis(5)) > 0.3 && leftScissor.getSelectedSensorPosition(0) >= MAX_HEIGHT) {
			leftScissor.set(ControlMode.PercentOutput, operator.getRawAxis(5));
		} else {
			leftScissor.set(ControlMode.PercentOutput, 0);
		}

		/*
		 * Scissor lift on POV levels POV 0: loweest (pick up, vault running) POV 90:
		 * switch level POV 180: scale (highest 6ft) POV 270: climb
		 */
		if (operator.getPOV() == 0) {
			leftScissor.set(ControlMode.Position, 0);
			SmartDashboard.putString("Scissor Lift", "Ground Level");
		} else if (operator.getPOV() == 90) {
			leftScissor.set(ControlMode.Position, SWITCH);
			SmartDashboard.putString("Scissor Lift", "Switch Level");
		} else if (operator.getPOV() == 180) {
			leftScissor.set(ControlMode.Position, SCALE);
			SmartDashboard.putString("Scissor Lift", "Scale Level");
		} else if (operator.getPOV() == 270) {
			leftScissor.set(ControlMode.Position, CLIMB);
			SmartDashboard.putString("Scissor Lift", "Climb Level");
		}
	}

	// right joystick up down
	private void deployArm() {
		deploy.set(-operator.getRawAxis(1) * 0.6);
	}

	/*
	 * controls the winch. back button down start button up
	 * 
	 */
	private void winchControl() {
		if (operator.getStartButton()) {
			winch.set(.5);
		} else if (operator.getBackButton()) {
			winch.set(-.5);
		} else {
			winch.set(0);
		}

	}

	/*
	 * intake lift controls the wheeled intake. right bumper button goes up, left
	 * bumper button holds in place.
	 */
	private void intakeLiftControl() {
		if (operator.getRawButton(5)) {
			intakeLift.set(-0.8);
		} else if (operator.getRawButton(6)) {
			intakeLift.set(-0.4);
		} else {
			intakeLift.set(0);
		}
	}

}
