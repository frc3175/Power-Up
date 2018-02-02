/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team3175.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private Victor leftArm = new Victor(4);

	// Victor scissor lift
	private TalonSRX scissorLift = new TalonSRX(0);

	// right gripping arm
	private TalonSRX rightArm = new TalonSRX(1);

	// Gyroscope
	private Gyro gyro = new ADXRS450_Gyro();

	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		driveTrain = new DifferentialDrive(leftDrive, rightDrive);
		scissorLift.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative,
				0, 0);
		rightArm.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.CTRE_MagEncoder_Relative, 1,
				0);
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

	@Override
	public void teleopPeriodic() {
		// test gyro
		SmartDashboard.putNumber("Gyro", gyro.getAngle());
		System.out.println("Here's your angle: " + gyro.getAngle());

		// test encoder
		SmartDashboard.putNumber("Scissor lift position: ", scissorLift.getSelectedSensorPosition(0) / 4096);
		SmartDashboard.putNumber("Right arm position: ", rightArm.getSelectedSensorPosition(0) / 4096);

		// tank drive
		// driveTrain.tankDrive(driver.getRawAxis(1), driver.getRawAxis(5));

		// arcade drive
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

		// intake arm
		if (operator.getXButton()) {
			leftArm.set(0.3);
			rightArm.set(ControlMode.Velocity, 0.3);
		} else if (operator.getYButton()) {
			leftArm.set(-0.3);
			rightArm.set(ControlMode.Velocity, -0.3);
		} else {
			leftArm.set(0);
			rightArm.set(ControlMode.Velocity, 0);
		}

		// scissor lift right operator stick y
		scissorLift.set(ControlMode.Velocity, operator.getRawAxis(5) * 0.5);
	}

}