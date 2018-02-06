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

	private Timer runTime = new Timer();

	// Joystick
	private Joystick driver = new Joystick(0);
	public XboxController operator = new XboxController(1);
	public XboxController driverController = new XboxController(0);
	// Drive train
	private Victor leftDrive = new Victor(0);
	private Victor rightDrive = new Victor(1);
	private DifferentialDrive driveTrain;

	// winch
	private Victor winch = new Victor(2);

	// Victor intakes
	private Victor intake = new Victor(3);

	// left gripping arm
	private Victor leftArm = new Victor(4);

	// Victor scissor lift
	private TalonSRX scissorLift = new TalonSRX(5);

	// right gripping arm
	private TalonSRX rightArm = new TalonSRX(6);

	private Victor testScissor = new Victor(7);

	// encoder counters
	private static int scissorPos;
	private static int armPos;

	// Gyroscope
	private Gyro gyro = new ADXRS450_Gyro();

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;

	private VisionThread visionThread;
	private double centerX = 0.0;

	private final Object imgLock = new Object();

	@Override
	public void robotInit() {
		driveTrain = new DifferentialDrive(leftDrive, rightDrive);
		scissorLift.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative,
				0, 0);
		rightArm.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative, 0,
				0);
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
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		gyro.calibrate();
		Timer.delay(0.05);
		runTime.reset();
		runTime.start();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		if (runTime.get() < 2) {
			// Invert the direction of the turn if we are going backwards
			driveTrain.arcadeDrive(0.4, 0);
		} else if (gyro.getAngle() < 90) {
			driveTrain.arcadeDrive(0.4, 0.5);
		}

		// } else if (runTime.get() > 2 && runTime.get() < 4.8) {
		// gyro.reset();
		//
		// // Invert the direction of the turn if we are going backwards
		// driveTrain.arcadeDrive(0.4, 0.45);
		// }

		// String gameData;
		// gameData = DriverStation.getInstance().getGameSpecificMessage();
		// if (gameData.charAt(0) == 'L') {
		// // Put left auto code here
		// } else {
		// // Put right auto code here
		// }

	}

	/**
	 * This function is run once each time the robot enters teleop mode.
	 */
	@Override
	public void teleopInit() {
		scissorPos = Math.abs(scissorLift.getSelectedSensorPosition(0));
		armPos = Math.abs(rightArm.getSelectedSensorPosition(0));
	}

	@Override
	public void teleopPeriodic() {
		// test gyro
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		// System.out.println("Here's your angle: " + gyro.getAngle());

		// test encoder
		SmartDashboard.putNumber("Scissor lift position: ", Math.abs(scissorLift.getSelectedSensorPosition(0) / 4096));
		SmartDashboard.putNumber("Right arm position: ", Math.abs(rightArm.getSelectedSensorPosition(0) / 4096));

		// tank drive
		// driveTrain.tankDrive(driver.getRawAxis(1), driver.getRawAxis(5));

		// arcade drive
		double turnSpeed;
		if (driver.getRawAxis(0) > 0.3 || driver.getRawAxis(0) < -0.3) {
			turnSpeed = -driver.getRawAxis(0) * 0.8;
		} else {
			turnSpeed = 0;
		}
		driveTrain.arcadeDrive(driver.getRawAxis(1), turnSpeed);

		// Gears of the drive train left joystick
		driveTrain.arcadeDrive(driver.getRawAxis(1), turnSpeed);
		// When A is held the the driveTrain goes 80%
		while (driverController.getAButton()) { // Listens for A button
			driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.8, turnSpeed * 0.8);
			// While A button is held it executes the normal code at 80%
		}
		// When B is held the driveTrain goes 60%
		while (driverController.getBButton()) {
			driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.6, turnSpeed * 0.6);
			// While B button is held it executes the normal code at 60%
		}
		// When Y is held the driveTrain goes 40%
		while (driverController.getYButton()) {
			driveTrain.arcadeDrive(driver.getRawAxis(1) * 0.4, turnSpeed * 0.4);
			// While Y button is held it executes the normal code at 40%
		}

		// Operator Stick Intakes
		if (operator.getAButton()) {
			intake.set(1); // A spit out
		} else if (operator.getBButton()) {
			intake.set(-0.5); // B intake
		} else {
			intake.set(0); // stop motor
		}
		// Winch with start and back buttons
		if (operator.getStartButton()) {
			winch.set(0.5);
		} else if (operator.getBackButton()) {
			winch.set(-0.5);
		} else {
			winch.set(0);
		}
		// intake arm left trigger in right trigger out
		if (operator.getRawButton(5)) {
			if (Math.abs((Math.abs(rightArm.getSelectedSensorPosition(0)) - armPos)) / 4096 <= 1) {
				leftArm.set(0.3);
				rightArm.set(ControlMode.PercentOutput, 0.3);
			}
			while (Math.abs((Math.abs(rightArm.getSelectedSensorPosition(0)) - armPos)) / 4096 <= 1) {
				System.out.println("Arm in");
			}
			leftArm.set(0);
			rightArm.set(ControlMode.PercentOutput, 0);
		} else if (operator.getRawButton(6)) {
			if (Math.abs((rightArm.getSelectedSensorPosition(0)) - armPos) >= 0) {
				rightArm.set(ControlMode.PercentOutput, -0.3);
			}
			while ((Math.abs(rightArm.getSelectedSensorPosition(0)) - armPos) >= 0) {
				System.out.println("Arm Out");
			}
			leftArm.set(-0.3);
			rightArm.set(ControlMode.PercentOutput, -0.3);
		} else {
			leftArm.set(0);
			rightArm.set(ControlMode.PercentOutput, 0);
		}

		// scissor lift right operator y up x down
		if (operator.getYButton()) {
			if (Math.abs((Math.abs(scissorLift.getSelectedSensorPosition(0)) - scissorPos)) / 4096 <= 5) {
				scissorLift.set(ControlMode.PercentOutput, 0.3);
			}
			while ((Math.abs(scissorLift.getSelectedSensorPosition(0)) - scissorPos) / 4096 <= 5) {
				System.out.println("Going up");
			}
			scissorLift.set(ControlMode.Velocity, 0);
		}
		if (operator.getXButton()) {
			System.out.println("X");
			if ((Math.abs(scissorLift.getSelectedSensorPosition(0)) - scissorPos) >= 0) {
				scissorLift.set(ControlMode.PercentOutput, -0.3);
			}
			while ((Math.abs(scissorLift.getSelectedSensorPosition(0)) - scissorPos) >= 0) {
				System.out.println("Going down");
			}
			scissorLift.set(ControlMode.Velocity, 0);
		}

		testScissor.set(operator.getRawAxis(5));

		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		System.out.println(centerX);
	}

}
