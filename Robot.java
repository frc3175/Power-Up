
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3175.robot;

import org.opencv.core.KeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.UsbCamera;
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
import edu.wpi.first.wpilibj.vision.VisionThread;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with arcade drive.
 */
public class Robot extends IterativeRobot {

	// scissor lift preset heights
	private static double SWITCH = 1.0;
	private static double SCALE = 3.0;
	private static double CLIMB = 4.0;

	/** SET THIS BEFORE MATCH! **/
	public String goal = "switch"; // TODO: Get DS input?

	// arcade drive speeds
	private static int gear = 1;

	private Timer runTime = new Timer();

	// Joystick
	private Joystick driver;
	private XboxController operator;
	public XboxController driverController;

	// Drive train
	private Victor leftDrive;
	private Victor rightDrive;
	private DifferentialDrive driveTrain;

	// lift system
	private Victor winch;
	private TalonSRX leftScissor;
	private Victor rightScissor;

	// manipulators
	private Victor intake;

	// pneumatics
	private DoubleSolenoid arm;
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

	// vission processing
	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	private VisionThread visionThread;
	private double centerX = 0.0;
	private final Object imgLock = new Object();	

	/**
	 * This function is run once each time the robot turns on.
	 */
	@Override
	public void robotInit() {
		SmartDashboard.putString("Robot Init", "Initializing...");

		driver = new Joystick(0);
		operator = new XboxController(1);
		driverController = new XboxController(0);
		leftDrive = new Victor(0);
		rightDrive = new Victor(1);
		winch = new Victor(2);
		intake = new Victor(3);
		rightScissor = new Victor(4);
		leftScissor = new TalonSRX(5);
		arm = new DoubleSolenoid(4, 5);
		deploy = new Solenoid(1);
		compressor = new Compressor(0);
		gyro = new ADXRS450_Gyro();

		compressor.setClosedLoopControl(true);

		gyro.calibrate();
		Timer.delay(0.05);
		gyro.reset();

		driveTrain = new DifferentialDrive(leftDrive, rightDrive);

		leftScissor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative,
				0, 0);

		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		visionThread = new VisionThread(camera, new findBlock(), pipeline -> {
			if (!pipeline.findBlobsOutput().empty()) {
				KeyPoint[] refKp = pipeline.findBlobsOutput().toArray();
				Point[] refPts = new Point[2];
				for (int i = 0; i < 2; i++) {
					refPts[i] = refKp[i].pt;
				}
				MatOfPoint2f refMatPt = new MatOfPoint2f(refPts);
				MatOfPoint2f approxCurve = new MatOfPoint2f();

				// Processing on mMOP2f1 which is in type MatOfPoint2f
				double approxDistance = Imgproc.arcLength(refMatPt, true) * 0.02;
				Imgproc.approxPolyDP(refMatPt, approxCurve, approxDistance, true);

				// Convert back to MatOfPoint
				MatOfPoint points = new MatOfPoint(approxCurve.toArray());
				// Get bounding rect
				Rect r = Imgproc.boundingRect(points);
				synchronized (imgLock) {
					centerX = r.x + (r.width / 2);
				}
			}
		});
		visionThread.start();

		runTime.reset();

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
		// driveTrain.arcadeDrive(0.4, 0);

		deploy.set(true);
		switch (station) {
		case 1:
			if (field.charAt(0) == 'L' && goal == "switch") {
				// alliance station 1 (left) switch on the left
				// Goes straight to the switch and puts cube in

			} else if (field.charAt(0) == 'R' && goal == "switch") {
				// alliance station 1 (left) switch on the right
				// Goes the long way around the switch (avoid collisions)

			} else if (field.charAt(1) == 'L' && goal == "scale") {
				// alliance station 1 (left) scale on the left
				// Goes the long way around the switch (avoid collisions)

			} else if (field.charAt(1) == 'R' && goal == "scale") {
				// alliance station 1 (left) scale on the right
				// Goes the long way around the switch (avoid collisions)

			}
			break;
		case 2:

			break;
		case 3:
			break;
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

		SmartDashboard.putString("Teleop Init", "Initialized!");
	}

	/**
	 * This function is called periodically during teleop.
	 */
	@Override
	public void teleopPeriodic() {

		telemetry();

		arcadeDrive();

		// operator x closes up y opens
		if (operator.getXButton()) {
			arm.set(DoubleSolenoid.Value.kReverse);
		} else if (operator.getYButton()) {
			arm.set(DoubleSolenoid.Value.kForward);
		}

		// Operator Stick Intakes
		if (operator.getAButton()) {
			intake.set(1); // A spit out
		} else if (operator.getBButton()) {
			intake.set(-0.5); // B intake
		} else {
			intake.set(0); // stop motor
		}

		// winch back button down start button up
		if (operator.getStartButton()) {
			winch.set(1);
		} else if (operator.getBackButton()) {
			winch.set(-1);
		} else {
			winch.set(0);
		}

		// scissor lift right joystick y override
		leftScissor.set(ControlMode.PercentOutput, operator.getRawAxis(5));
		rightScissor.set(operator.getRawAxis(5));

		/*
		 * Scissor lift on POV levels POV 0: loweest (pick up, vault running)
		 * POV 90: switch level POV 180: scale (highest 6ft) POV 270: climb
		 */
		if (operator.getPOV() == 0) {
			groundLevel();
		} else if (operator.getPOV() == 90) {
			switchLevel();
		} else if (operator.getPOV() == 180) {
			scaleLevel();
		} else if (operator.getPOV() == 270) {
			climbLevel();
		}

		// test vision
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		SmartDashboard.putNumber("Center X", centerX);
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
		SmartDashboard.putString("Gear", Integer.toString(gear));
		if (gear == 1) {
			SmartDashboard.putString("Gear", "40% Speed");
		} else if (gear == 2) {
			SmartDashboard.putString("Gear", "60% Speed");
		} else if (gear == 3) {
			SmartDashboard.putString("Gear", "80% Speed");
		} else if (gear == 0) {
			SmartDashboard.putString("Gear", "100% Speed");
		}

		// runtime
		SmartDashboard.putString("Run Time", Double.toString(runTime.get()));
	}

	/*
	 * Programmable four speed arcade drive
	 */
	private void arcadeDrive() {
		double turnSpeed;
		if (driver.getRawAxis(0) > 0.3 || driver.getRawAxis(0) < -0.3) {
			turnSpeed = -driver.getRawAxis(0);
		} else {
			turnSpeed = 0;
		}

		// Gears of the drive train left joystick
		driveTrain.arcadeDrive(driver.getRawAxis(1), turnSpeed);
		gear = 0;
		// When A is held the the driveTrain goes 80%
		if (driverController.getAButton()) { // Listens for A button
			driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.8, turnSpeed * 0.8);
			gear = 1;
			// While A button is held it executes the normal code at 80%
		}
		// When B is held the driveTrain goes 60%
		if (driverController.getBButton()) {
			driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.6, turnSpeed * 0.6);
			gear = 2;
			// While B button is held it executes the normal code at 60%
		}
		// When Y is held the driveTrain goes 40%
		if (driverController.getYButton()) {
			driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.4, turnSpeed * 0.4);
			gear = 3;
			// While Y button is held it executes the normal code at 40%
		}

		// trigger buttons shift gears
		if (driver.getRawButton(5) && gear > 1) {
			// left gear down
			gear--;
		} else if (driver.getRawButton(6) && gear < 4) {
			// right gear up
			gear++;
		}
	}

	/*
	 * drives the scissor lift to the lowest position
	 */
	private void groundLevel() {
		if (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 > 0) {
			// go down when it is higher than the lowest level
			leftScissor.set(ControlMode.PercentOutput, -0.5);
			rightScissor.set(-0.5);
			double timeOut = runTime.get();
			while (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 > 0 && (runTime.get() - timeOut) < 5) {
				SmartDashboard.putString("Going down", "Ground Level");
			}
			leftScissor.set(ControlMode.Velocity, 0);
			rightScissor.set(0);
		}
	}

	/*
	 * drives the scissor lift to the switch level
	 */
	private void switchLevel() {
		if (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 < SWITCH) {
			// go up when it is lower than switch level
			leftScissor.set(ControlMode.PercentOutput, 0.5);
			rightScissor.set(0.5);
			double timeOut = runTime.get();
			while (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 <= SWITCH
					&& (runTime.get() - timeOut) < 5) {
				SmartDashboard.putString("Going up", "Switch");
			}
			leftScissor.set(ControlMode.Velocity, 0);
			rightScissor.set(0);
		} else if (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 > SWITCH) {
			// go down when it is higher than switch level
			leftScissor.set(ControlMode.PercentOutput, -0.5);
			rightScissor.set(-0.5);
			double timeOut = runTime.get();
			while (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 > SWITCH
					&& (runTime.get() - timeOut) < 5) {
				SmartDashboard.putString("Going down", "Switch");
			}
			leftScissor.set(ControlMode.Velocity, 0);
			rightScissor.set(0);
		}
	}

	/*
	 * drives the scissor lift to the scale level
	 */
	private void scaleLevel() {
		if (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 < SCALE) {
			// go up when it is lower than the scale level
			leftScissor.set(ControlMode.PercentOutput, 0.5);
			rightScissor.set(0.5);
			double timeOut = runTime.get();
			while (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 <= SCALE
					&& (runTime.get() - timeOut) < 5) {
				SmartDashboard.putString("Going up", "Scale");
			}
			leftScissor.set(ControlMode.Velocity, 0);
			rightScissor.set(0);
		} else if (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 > SCALE) {
			// go down when it is higher than the scale level
			leftScissor.set(ControlMode.PercentOutput, -0.5);
			rightScissor.set(-0.5);
			double timeOut = runTime.get();
			while (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 > SCALE
					&& (runTime.get() - timeOut) < 5) {
				SmartDashboard.putString("Going down", "Scale");
			}
			leftScissor.set(ControlMode.Velocity, 0);
			rightScissor.set(0);
		}
	}

	/*
	 * drives the scissor lift to the climber level
	 */
	private void climbLevel() {
		if (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 < CLIMB) {
			// go up when it is lower than switch level
			leftScissor.set(ControlMode.PercentOutput, 0.5);
			rightScissor.set(0.5);
			double timeOut = runTime.get();
			while (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 <= CLIMB
					&& (runTime.get() - timeOut) < 5) {
				SmartDashboard.putString("Going up", "Climb");
			}
			leftScissor.set(ControlMode.Velocity, 0);
			rightScissor.set(0);
		}
	}

}
