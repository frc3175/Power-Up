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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
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
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {

	// scissor lift preset heights
	private static double SWITCH = 1.0;
	private static double SCALE = 3.0;
	private static double CLIMB = 4.0;

	/** SET THIS BEFORE MATCH! **/
	public String goal = "switch"; // TODO: Get DS input?

	private Timer runTime = new Timer();

	private double currentTime;

	// Joystick
	private Joystick driver;
	private XboxController operator;

	// Drive train
	private Victor leftDrive;
	private Victor rightDrive;
	private DifferentialDrive driveTrain;

	// winch
	private Victor winch;

	// Victor intakes
	private Victor intake;

	// left gripping arm
	private Victor rightArm;

	// Victor scissor lift
	private TalonSRX leftScissor;

	// right gripping arm
	private TalonSRX leftArm;

	private Victor rightScissor;

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
		leftDrive = new Victor(0);
		rightDrive = new Victor(1);
		winch = new Victor(2);
		intake = new Victor(3);
		rightArm = new Victor(4);
		leftScissor = new TalonSRX(5);
		leftArm = new TalonSRX(6);
		rightScissor = new Victor(7);
		gyro = new ADXRS450_Gyro();

		driveTrain = new DifferentialDrive(leftDrive, rightDrive);

		leftScissor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative,
				0, 0);
		leftArm.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative, 0,
				0);
		leftArm.setSelectedSensorPosition(0, 0, 10);

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

		leftArm.setSelectedSensorPosition(0, 0, 10);
		leftScissor.setSelectedSensorPosition(0, 0, 10);
		field = DriverStation.getInstance().getGameSpecificMessage();
		gyro.calibrate();
		Timer.delay(5);
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

		gyro.calibrate();
		Timer.delay(5);
		gyro.reset();
		leftArm.setSelectedSensorPosition(0, 0, 10);
		leftScissor.setSelectedSensorPosition(0, 0, 10);

		SmartDashboard.putString("Teleop Init", "Initialized!");
	}

	/**
	 * This function is called periodically during teleop.
	 */
	@Override
	public void teleopPeriodic() {

		// gyro reading
		SmartDashboard.putNumber("Gyro", gyro.getAngle());

		// encoder readings
		SmartDashboard.putNumber("Scissor lift position: ",
				Math.abs(leftScissor.getSelectedSensorPosition(0) / 4096.0));
		SmartDashboard.putNumber("Left Arm position: ", Math.abs(leftArm.getSelectedSensorPosition(0) / 4096.0));

		// arcade drive;
		double turnSpeed;
		if (driver.getRawAxis(4) > 0.3 || driver.getRawAxis(4) < -0.3) {
			turnSpeed = -driver.getRawAxis(4);
		} else {
			turnSpeed = 0;
		}
		if (driver.getRawButton(4)) {
			// y 40% speed
			driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.4, turnSpeed * 0.4);
		} else if (driver.getRawButton(3)) {
			// b 60% speed
			driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.6, turnSpeed * 0.6);
		} else if (driver.getRawButton(2)) {
			// a 80% speed
			driveTrain.arcadeDrive(driver.getRawAxis(5) * 0.8, turnSpeed * 0.8);
		} else {
			driveTrain.arcadeDrive(driver.getRawAxis(5), turnSpeed);
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

		// intake arm x grabs y releases
		if (operator.getXButton()) {
			if (Math.abs(leftArm.getSelectedSensorPosition(0)) / 4096.0 <= 0.3) {
				rightArm.set(-0.3);
				leftArm.set(ControlMode.PercentOutput, 0.3);
			}
			double timeOut = runTime.get();
			while (Math.abs(leftArm.getSelectedSensorPosition(0)) / 4096.0 <= 0.3 && (runTime.get() - timeOut) < 0.5) {
				System.out.println("Arm in");
			}
			rightArm.set(0);
			leftArm.set(ControlMode.PercentOutput, 0);
		} else if (operator.getYButton()) {
			if (Math.abs(leftArm.getSelectedSensorPosition(0)) >= 0) {
				rightArm.set(-0.3);
				leftArm.set(ControlMode.PercentOutput, 0.3);
			}
			double timeOut = runTime.get();
			while (Math.abs(leftArm.getSelectedSensorPosition(0)) >= 0 && (runTime.get() - timeOut) < 0.5) {
				System.out.println("Arm Out");
			}
			rightArm.set(0);
			leftArm.set(ControlMode.PercentOutput, 0);
		} else {
			rightArm.set(0);
			leftArm.set(ControlMode.PercentOutput, 0);
		}

		// scissor lift right joystick y override
		leftScissor.set(ControlMode.PercentOutput, operator.getRawAxis(5));
		rightScissor.set(operator.getRawAxis(5));

		/*
		 * Scissor lift on POV levels POV 0: loweest (pick up, vault running) POV 90:
		 * switch level POV 180: scale (highest 6ft) POV 270: climb
		 */
		if (operator.getPOV() == 0) {
			if (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 > 0) {
				// go down when it is higher than the lowest level
				leftScissor.set(ControlMode.PercentOutput, -0.5);
				rightScissor.set(-0.5);
				double timeOut = runTime.get();
				while (Math.abs(leftScissor.getSelectedSensorPosition(0)) / 4096.0 > 0
						&& (runTime.get() - timeOut) < 5) {
					SmartDashboard.putString("Going down", "Ground Level");
				}
				leftScissor.set(ControlMode.Velocity, 0);
				rightScissor.set(0);
			}
		} else if (operator.getPOV() == 90) {
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
		} else if (operator.getPOV() == 180) {
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
		} else if (operator.getPOV() == 270) {
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

		// test vision
		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		SmartDashboard.putNumber("Center X", centerX);

		System.out.println(runTime.get());
	}

}
