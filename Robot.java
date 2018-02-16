/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3175.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
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
	private static final double SWITCH = 6.0 * 4096;
	private static final double SCALE = 15.0 * 4096;
	private static final double CLIMB = 17.0 * 4096;
	// private static final double MAX_HEIGHT = 18.5;

	private int level = 0;
	private boolean deployed = false;

	/** SET THIS BEFORE MATCH! **/
	public String goal = "switch";

	// arcade drive speeds
	private static int gear = 1;

	private Timer runTime = new Timer();

	// Joystick
	private Joystick driver;
	private XboxController operator;
	public XboxController driverController; // Ian drives

	// Drive train
	private Victor leftDrive;
	private Victor rightDrive;
	private DifferentialDrive driveTrain;

	// lift system
	private Victor winch;
	private TalonSRX leftScissor;

	private Victor intake = new Victor(3);;

	// pneumatics
	private DoubleSolenoid intakeArm;
	private Solenoid deploy;
	private Compressor compressor;

	// Gyroscope
	private Gyro gyro;

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

		driver = new Joystick(0);
		driverController = new XboxController(0);
		operator = new XboxController(1);

		leftDrive = new Victor(0);
		rightDrive = new Victor(1);
		driveTrain = new DifferentialDrive(leftDrive, rightDrive);

		winch = new Victor(2);
		leftScissor = new TalonSRX(5);
		leftScissor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative,
				0, 10);
		// /*
		// * Talon configured to have soft limits 10000 native units in either direction
		// * and enabled
		// */
		// leftScissor.configForwardSoftLimitThreshold(0, 0);
		// leftScissor.configReverseSoftLimitThreshold((int) (4096 * MAX_HEIGHT), 0);
		// leftScissor.configForwardSoftLimitEnable(true, 0);
		// leftScissor.configReverseSoftLimitEnable(true, 0);
		/* set closed loop gains in slot0, typically kF stays zero. */
		leftScissor.config_kF(0, 0.0, 10);
		leftScissor.config_kP(0, 0.1, 10);
		leftScissor.config_kI(0, 0.0, 10);
		leftScissor.config_kD(0, 0.0, 10);

		intakeArm = new DoubleSolenoid(4, 5);
		deploy = new Solenoid(6);
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(false);

		gyro = new ADXRS450_Gyro();
		gyro.calibrate();
		Timer.delay(5);
		gyro.reset();

		CameraServer.getInstance().startAutomaticCapture();

		alliance = DriverStation.getInstance().getAlliance();
		station = DriverStation.getInstance().getLocation();
		SmartDashboard.putString("Alliance", alliance + Integer.toString(station));
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

		gyro.reset();
		runTime.reset();
		runTime.start();

		SmartDashboard.putString("Auton Init", "Initialized!");
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		if (!deployed) {
			deploy.set(true);
			if (runTime.get() >= 0.5) {
				deployed = true;
				deploy.set(false);
			}
		}
		if (!goal.equals("cross") && runTime.get() > 0.5) {
			switch (station) {
			case 1: // alliance station 1 (left)
				if (field.charAt(0) == 'L' && goal == "switch") {
					// switch on the left Goes straight to the switch and puts cube in
					if (runTime.get() < 1.8) {
						driveTrain.arcadeDrive(0.75, 0);
					}

				} else if (field.charAt(0) == 'R' && goal.equals("switch")) {
					// switch on the right Goes the long way around the switch (avoid collisions)

				} else if (field.charAt(1) == 'L' && goal.equals("scale")) {
					// scale on the left drive forward lift and drop at the scale

				} else if (field.charAt(1) == 'R' && goal.equals("scale")) {
					// scale on the right drive forward turn right

				}
				break;
			case 2: // alliance station 2 (middle)
				if (field.charAt(0) == 'L' && goal.equals("switch")) {
					// switch on the left turns left and put the block

				} else if (field.charAt(0) == 'R' && goal.equals("switch")) {
					// switch on the right turns right and put the block

				} else if (field.charAt(1) == 'L' && goal.equals("scale")) {
					// scale on the left turn left and go the long way around the switch

				} else if (field.charAt(1) == 'R' && goal.equals("scale")) {
					// scale on the right turn right and go the long way around the switch

				}
				break;
			case 3: // alliance station 3 (right)
				if (field.charAt(0) == 'L' && goal.equals("switch")) {
					// switch on the left Goes the long way around the switch (avoid collisions)

				} else if (field.charAt(0) == 'R' && goal.equals("switch")) {
					// switch on the right Goes straight to the switch and puts cube in

				} else if (field.charAt(1) == 'L' && goal.equals("scale")) {
					// scale on the left go forward turn left

				} else if (field.charAt(1) == 'R' && goal.equals("scale")) {
					// scale on the right go forward to the scale

				}
				break;
			}
		} else if (goal.equals("cross") && runTime.get() > 0.5) {
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

		gyro.reset();
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
		armControl();
		winchControl();
		scissorControl();

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
	 * Sends robot data to the SmartDashboard
	 */
	private void telemetry() {
		// gyro reading
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		// encoder readings
		SmartDashboard.putNumber("Scissor lift position: ",
				Math.abs(leftScissor.getSelectedSensorPosition(0) / 4096.0));
		// pneumatics
		SmartDashboard.putBoolean("Compressor enabled", compressor.enabled());
		SmartDashboard.putBoolean("Pressure Switch On", compressor.getPressureSwitchValue());
		// drive train
		if (gear == 1) {
			SmartDashboard.putString("Gear", "40% Speed");
		} else if (gear == 2) {
			SmartDashboard.putString("Gear", "60% Speed");
		} else if (gear == 3) {
			SmartDashboard.putString("Gear", "80% Speed");
		} else if (gear == 4) {
			SmartDashboard.putString("Gear", "100% Speed");
		}
		// runtime
		SmartDashboard.putString("Run time", Double.toString(runTime.get()));
	}

	/*
	 * Programmable four speed arcade drive
	 */
	private void drive() {
		double turnSpeed;
		if (operator != null) {
			if (Math.abs(driver.getRawAxis(4)) > 0.3) {
				turnSpeed = -driver.getRawAxis(4);
			} else {
				turnSpeed = 0;
			}
			switch (gear) {
			case 1:
				// gear 1 40% speed
				driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.4, turnSpeed * 0.4);
				break;
			case 2:
				// gear 2 60% speed
				driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.6, turnSpeed * 0.6);
				break;
			case 3:
				// gear 3 80% speed
				driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.8, turnSpeed * 0.8);
				break;
			case 4:
				// gear 4 100% speed
				driveTrain.arcadeDrive(driver.getRawAxis(5), turnSpeed);
				break;
			}

			// trigger buttons shift gears
			if (driver.getRawButton(5) && gear > 1) {
				// left gear down
				gear--;
			} else if (driver.getRawButton(6) && gear < 4) {
				// right gear up
				gear++;
			}
		} else {
			if (driverController.getRawAxis(0) > 0.3 || driverController.getRawAxis(0) < -0.3) {
				turnSpeed = -driverController.getRawAxis(0);
			} else {
				turnSpeed = 0;
			}
			// Gears of the drive train left joystick
			if (driverController.getXButton()) {
				gear = 4;
			} else if (driverController.getAButton()) { // Listens for A button
				// While A button is held it executes the normal code at 80%
				gear = 3;
			} else if (driverController.getBButton()) {
				// While B button is held it executes the normal code at 60%
				gear = 2;
			} else if (driverController.getYButton()) {
				// While Y button is held it executes the normal code at 40%
				gear = 1;
			}

			switch (gear) {
			case 1:
				// gear 1 40% speed
				driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.4, turnSpeed * 0.4);
				break;
			case 2:
				// gear 2 60% speed
				driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.6, turnSpeed * 0.6);
				break;
			case 3:
				// gear 3 80% speed
				driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.8, turnSpeed * 0.8);
				break;
			case 4:
				// gear 4 100% speed
				driveTrain.arcadeDrive(driver.getRawAxis(5), turnSpeed);
				break;
			}
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
	 * controls the winch. back button down start button up
	 * 
	 */
	private void winchControl() {
		if (operator.getStartButton()) {
			winch.set(1);
		} else if (operator.getBackButton()) {
			winch.set(-1);
		} else {
			winch.set(0);
		}

	}

	/*
	 * controls the movement of the scissor lift.
	 */
	private void scissorControl() {
		// scissor lift right joystick y override
		if (Math.abs(operator.getRawAxis(5)) > 0.3) {
			leftScissor.set(ControlMode.PercentOutput, operator.getRawAxis(5));
		}

		/*
		 * Scissor lift on POV levels POV 0: loweest (pick up, vault running) POV 90:
		 * switch level POV 180: scale (highest 6ft) POV 270: climb
		 */
		if (operator.getPOV() == 0) {
			level = 0;
			SmartDashboard.putString("Scissor Lift", "Ground Level");
		} else if (operator.getPOV() == 90) {
			level = 1;
			SmartDashboard.putString("Scissor Lift", "Switch Level");
		} else if (operator.getPOV() == 180) {
			level = 2;
			SmartDashboard.putString("Scissor Lift", "Scale Level");
		} else if (operator.getPOV() == 270) {
			level = 3;
			SmartDashboard.putString("Scissor Lift", "Climb Level");
		}

		if (level == 0) {
			leftScissor.set(ControlMode.Position, 0);
		} else if (level == 1) {
			leftScissor.set(ControlMode.Position, SWITCH * 4096.0);
		} else if (level == 2) {
			leftScissor.set(ControlMode.Position, SCALE * 4096.0);
		} else if (level == 3) {
			leftScissor.set(ControlMode.Position, CLIMB * 4096.0);
		}
	}

}
