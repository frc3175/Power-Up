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

	// Drive train
	private Victor leftDrive = new Victor(0);
	private Victor rightDrive = new Victor(1);
	private DifferentialDrive driveTrain;

	// winch
	private Victor winch = new Victor(2);

	// Victor intakes
	private Victor intake = new Victor(3);

	// left gripping arm
	private Victor rightArm = new Victor(4);

	// Victor scissor lift
	private TalonSRX scissorLift = new TalonSRX(5);

	// right gripping arm
	private TalonSRX leftArm = new TalonSRX(6);

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
		gyro.calibrate();
		Timer.delay(5);
		gyro.reset();
		driveTrain = new DifferentialDrive(leftDrive, rightDrive);
		scissorLift.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative,
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
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		gyro.calibrate();
		Timer.delay(5);
		gyro.reset();
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
		armPos = Math.abs(leftArm.getSelectedSensorPosition(0));
	}

	@Override
	public void teleopPeriodic() {
		leftArm.setSelectedSensorPosition(0, 0, 10);

		// test gyro
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		// System.out.println("Here's your angle: " + gyro.getAngle());

		// test encoder
		SmartDashboard.putNumber("Scissor lift position: ",
				Math.abs(scissorLift.getSelectedSensorPosition(0) / 4096.0));
		SmartDashboard.putNumber("Left Arm position: ", Math.abs(leftArm.getSelectedSensorPosition(0) / 4096.0));

		// tank drive
		// driveTrain.tankDrive(driver.getRawAxis(1), driver.getRawAxis(5));

		// arcade drive;
		double turnSpeed;
		if (driver.getRawAxis(4) > 0.3 || driver.getRawAxis(4) < -0.3) {
			turnSpeed = -driver.getRawAxis(4) * 0.8;
		} else {
			turnSpeed = 0;
		}
		driveTrain.arcadeDrive(driver.getRawAxis(5), turnSpeed);

		// Operator Stick Intakes
		if (operator.getAButton()) {
			intake.set(1); // A spit out
		} else if (operator.getBButton()) {
			intake.set(-0.5); // B intake
		} else {
			intake.set(0); // stop motor
		}

		// intake arm x grabs y releases
		if (operator.getXButton()) {
			if (Math.abs((Math.abs(leftArm.getSelectedSensorPosition(0)) - armPos)) / 4096.0 <= 0.3) {
				rightArm.set(-0.3);
				leftArm.set(ControlMode.PercentOutput, 0.3);
			}
			double timeOut = runTime.get();
			while (Math.abs((Math.abs(leftArm.getSelectedSensorPosition(0)) - armPos)) / 4096.0 <= 0.3
					&& (runTime.get() - timeOut) < 0.3) {
				System.out.println("Arm in");
			}
			rightArm.set(0);
			leftArm.set(ControlMode.PercentOutput, 0);
		} else if (operator.getYButton()) {
			if (Math.abs((leftArm.getSelectedSensorPosition(0)) - armPos) >= 0) {
				leftArm.set(ControlMode.PercentOutput, 0.3);
				rightArm.set(-0.3);
			}
			double timeOut = runTime.get();
			while ((Math.abs(leftArm.getSelectedSensorPosition(0)) - armPos) >= 0 && (runTime.get() - timeOut) < 0.3) {
				System.out.println("Arm Out");
			}
			rightArm.set(0);
			leftArm.set(ControlMode.PercentOutput, 0);
		} else {
			rightArm.set(0);
			leftArm.set(ControlMode.PercentOutput, 0);
		}

		// scissor lift right operator y up x down
		if (operator.getYButton()) {
			if (Math.abs((Math.abs(scissorLift.getSelectedSensorPosition(0)) - scissorPos)) / 4096.0 <= 5) {
				scissorLift.set(ControlMode.PercentOutput, 0.3);
			}
			double timeOut = runTime.get();
			while ((Math.abs(scissorLift.getSelectedSensorPosition(0)) - scissorPos) / 4096.0 <= 5
					&& (runTime.get() - timeOut) < 5) {
				System.out.println("Going up");
			}
			scissorLift.set(ControlMode.Velocity, 0);
		}
		if (operator.getXButton()) {
			System.out.println("X");
			if ((Math.abs(scissorLift.getSelectedSensorPosition(0)) - scissorPos) >= 0) {
				scissorLift.set(ControlMode.PercentOutput, -0.3);
			}
			double timeOut = runTime.get();
			while ((Math.abs(scissorLift.getSelectedSensorPosition(0)) - scissorPos) >= 0
					&& (runTime.get() - timeOut) < 5) {
				System.out.println("Going down");
			}
			scissorLift.set(ControlMode.Velocity, 0);
		}

		testScissor.set(operator.getRawAxis(5));

		double centerX;
		synchronized (imgLock) {
			centerX = this.centerX;
		}
		SmartDashboard.putNumber("Center X", centerX);

		// if (operator.getXButton()) {
		// leftArm.set(ControlMode.PercentOutput, -0.5);
		// rightArm.set(0.5);
		// } else if (operator.getYButton()) {
		// leftArm.set(ControlMode.PercentOutput, 0.3);
		// rightArm.set(-0.3);
		// } else {
		// leftArm.set(ControlMode.PercentOutput, 0);
		// rightArm.set(0);
		// }
	}

}
