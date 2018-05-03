/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3175.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {

	/** SET THESE BEFORE MATCH! **/
	private static final int LOCATION = 1;
	// 0 left 1 right
	private static final String autonMode = "cross";
	// "cross" for crossing, "switch" for switch

	// the angle of the wheeled intake towards the switch
	private static final double INTAKE_SWITCH = -0.1 * 4096;
	// the angle of the wheeled intake at ground level
	private static final double INTAKE_GROUND = -0.32 * 4096;

	// drive speeds
	private static int gear = 1;

	private Timer runTime;

	// Joystick
	private Joystick driver;
	private XboxController operator;

	// Drive train
	private Victor leftDrive;
	private Victor rightDrive;
	private DifferentialDrive driveTrain;

	// lift system
	private Victor winch;
	private Victor hookDelivery;

	// wheeled intake
	private Victor intake;
	private TalonSRX intakeLift;

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
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		SmartDashboard.putString("Robot Init", "Initializing...");

		// select driver operator sticks
		driver = new Joystick(0);
		operator = new XboxController(1);

		// initializing drive train
		leftDrive = new Victor(0);
		rightDrive = new Victor(1);
		driveTrain = new DifferentialDrive(leftDrive, rightDrive);

		// initializing lift system
		winch = new Victor(2);
		hookDelivery = new Victor(3);

		// initialize intake system
		intake = new Victor(4);
		intakeLift = new TalonSRX(5);
		intakeLift.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative,
				0, 10);
		intakeLift.setSensorPhase(true);
		intakeLift.setNeutralMode(NeutralMode.Brake);
		/* set closed loop gains in slot0, typically kF stays zero. */
		intakeLift.config_kF(0, 0.0, 10);
		intakeLift.config_kP(0, 0.1, 10);
		intakeLift.config_kI(0, 0.0, 10);
		intakeLift.config_kD(0, 0.0, 10);

		intakeLift.setSelectedSensorPosition(0, 0, 10);

		// initialize pneumatics
		intakeArm = new DoubleSolenoid(4, 5);
		compressor = new Compressor(0);
		compressor.setClosedLoopControl(true);

		runTime = new Timer();

		SmartDashboard.putString("Robot Init", "Initialized!");
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		SmartDashboard.putString("Auton Init", "Initializing...");

		intakeLift.setSelectedSensorPosition(0, 0, 10);
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
		if (autonMode.equals("cross")) {
			if (runTime.get() < 2) {
				driveTrain.arcadeDrive(0.75, 0);
			}
		} else if (autonMode.equals("switch")) {
			if ((field.charAt(0) == 'L' && LOCATION == 0) || (field.charAt(0) == 'R' && LOCATION == 1)) {
				// 0 is left, 1 is right
				switchDump();
			} else {
				if (runTime.get() < 1.5) {
					driveTrain.arcadeDrive(0.75, 0);
				}
			}
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		telemetry();
		drive();
		intakeControl();
		winchControl();
		intakeLiftControl();
		hookControl();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

	/*
	 * Auton function drives close to the switch and shoots the block to the
	 * alliance switch
	 */
	private void switchDump() {
		if (runTime.get() < 1.5) {
			driveTrain.arcadeDrive(0.75, 0);
			// drive to switch and suddenly stops
			// } else if (runTime.get() < 2.5 && intakeLift.getSelectedSensorPosition(0) <
			// 4096.0 * INTAKE_SWITCH) {
			// intakeLift.set(ControlMode.PercentOutput, 0.5);
			// lower the intake to the switch level
		} else if (runTime.get() < 3) {
			intake.set(-0.75);
			// shoot the block
		} else {
			intake.set(0);
		}
	}

	/*
	 * Sends robot data to the SmartDashboard
	 */
	private void telemetry() {
		// encoder readings
		SmartDashboard.putNumber("Intake lift position: ", intakeLift.getSelectedSensorPosition(0) / 4096.0);
		// pneumatics
		SmartDashboard.putBoolean("Compressor enabled", compressor.enabled());
		SmartDashboard.putBoolean("Pressure Switch On", compressor.getPressureSwitchValue());
		// drive train
		if (gear == 1) {
			SmartDashboard.putString("Gear", "50% Speed");
		} else if (gear == 2) {
			SmartDashboard.putString("Gear", "80% Speed");
		} else if (gear == 3) {
			SmartDashboard.putString("Gear", "100% Speed");
		}
		// runtime
		SmartDashboard.putString("Run time", Double.toString(runTime.get()));
	}

	/*
	 * Programmable four speed arcade drive
	 */
	private void drive() {
		if (Math.abs(driver.getRawAxis(1) - driver.getRawAxis(5)) > 0.2) {
			driveTrain.tankDrive(-driver.getRawAxis(1), -driver.getRawAxis(5));
		} else {
			driveTrain.tankDrive(-Math.max(driver.getRawAxis(1), driver.getRawAxis(5)),
					-Math.max(driver.getRawAxis(1), driver.getRawAxis(5)));
		}
		gear = 3; // 100% speed
		if (driver.getRawButton(5)) {
			// left bumper button 80% speed
			if (Math.abs(driver.getRawAxis(1) - driver.getRawAxis(5)) > 0.2) {
				driveTrain.tankDrive(-driver.getRawAxis(1) * 0.8, -driver.getRawAxis(5) * 0.8);
			} else {
				driveTrain.tankDrive(-0.8 * Math.max(driver.getRawAxis(1), driver.getRawAxis(5)),
						-0.8 * Math.max(driver.getRawAxis(1), driver.getRawAxis(5)));
			}
			gear = 2;
		} else if (driver.getRawButton(6)) {
			// right bumper button% 50 speed
			if (Math.abs(driver.getRawAxis(1) - driver.getRawAxis(5)) > 0.2) {
				driveTrain.tankDrive(-driver.getRawAxis(1) * 0.5, -driver.getRawAxis(5) * 0.5);
			} else {
				driveTrain.tankDrive(-0.5 * Math.max(driver.getRawAxis(1), driver.getRawAxis(5)),
						-0.5 * Math.max(driver.getRawAxis(1), driver.getRawAxis(5)));
			}
			gear = 1;
		}
		if (driver.getPOV() == 0) {
			driveTrain.arcadeDrive(0.6, 0);
		} else if (driver.getPOV() == 180) {
			driveTrain.arcadeDrive(-0.6, 0);
		} else if (driver.getPOV() == 90) {
			driveTrain.arcadeDrive(0.3, 0.5);
		} else if (driver.getPOV() == 270) {
			driveTrain.arcadeDrive(0.3, -0.5);
		}
	}

	/*
	 * controls the wheeled intake and the intake lift
	 */
	private void intakeControl() {
		// Operator Stick Intakes
		if (operator.getBButton()) {
			intake.set(0.75); // B intakes
		} else if (operator.getAButton()) {
			intake.set(-0.8); // A spit out
		} else {
			intake.set(0); // stop motor
		}

		if (operator.getXButton()) {
			// x closes up
			intakeArm.set(DoubleSolenoid.Value.kReverse);
		} else if (operator.getYButton()) {
			// y opens
			intakeArm.set(DoubleSolenoid.Value.kForward);
		}
	}

	/*
	 * controls the winch. back button down start button up
	 * 
	 */
	private void winchControl() {
		if (operator.getStartButton()) {
			winch.set(-.5);
		} else if (operator.getBackButton()) {
			winch.set(.5);
		} else {
			winch.set(0);
		}
	}

	/*
	 * intake lift controls the angle of the wheeled intakes
	 */
	private void intakeLiftControl() {
		// intake lift right joystick y override
		if (Math.abs(operator.getRawAxis(5)) > 0.3) {
			intakeLift.set(ControlMode.PercentOutput, operator.getRawAxis(5) * 0.5);
		} else {
			intakeLift.set(ControlMode.PercentOutput, 0);
		}

		if (operator.getPOV() == 0) {
			// left bumper to switch level
			intakeLift.set(ControlMode.Position, INTAKE_SWITCH);
			SmartDashboard.putString("Intake Position: ", "Switch");
		} else if (operator.getPOV() == 180) {
			// right bumper to ground level
			intakeLift.set(ControlMode.Position, INTAKE_GROUND);
			SmartDashboard.putString("Intake Position: ", "Ground");
		}
	}

	/*
	 * controls the arm that delivers the hook
	 */
	private void hookControl() {
		// left joystick y
		if (Math.abs(operator.getRawAxis(1)) > 0.2) {
			hookDelivery.set(operator.getRawAxis(1) * 0.75);
		} else {
			hookDelivery.set(0);
		}
	}

}
