/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.commands.FieldCentricSwerveDrive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drive extends SubsystemBase {

	private static swerveModule frontLeft;
	private static swerveModule backLeft;
	private static swerveModule frontRight;
	private static swerveModule backRight;

	public static AHRS navX; 
    public double heading;
    public double angle;

	private static final double WHEEL_BASE_LENGTH = 22; // 28.0;
	private static final double WHEEL_BASE_WIDTH = 23.5; // 22.0;
	public static final double ENCODER_COUNT_PER_ROTATION = 4096.0;
	
	private static final double WHEEL_DIAMETER = 4.0;
	// TO DO: Correct equation that uses MAX_SPEED
	public static final double MAX_SPEED = 0.3; // Max speed is 0 to 1
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

	public static final double STEER_DEGREES_PER_COUNT = 360.0 / ENCODER_COUNT_PER_ROTATION;
	public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.63);
	public static final double DEADZONE = 0.08;

	public static final double OMEGA_SCALE = 1.0 / 30.0;

	private final boolean invertDrive = false;
	private final boolean invertSteer = true;

	public Drive() {
		
		// frontLeft = new setSwerveModule(Constants.FL_STEER_ENCODER, Constants.FL_STEER_MOTOR, 
		// Constants.FL_DRIVE_MOTOR, invertDrive, invertSteer);

		// backLeft = new setSwerveModule(Constants.BL_STEER_ENCODER, Constants.BL_STEER_MOTOR, 
		// Constants.BL_DRIVE_MOTOR, invertDrive, invertSteer);

		frontRight = new swerveModule(Constants.FR_STEER_ENCODER, Constants.FR_STEER_MOTOR, 
		Constants.FR_DRIVE_MOTOR, invertDrive, invertSteer);

		// backRight = new setSwerveModule(Constants.BR_STEER_ENCODER, Constants.BR_STEER_MOTOR, 
		// Constants.BR_DRIVE_MOTOR, invertDrive, invertSteer);	

		navX = new AHRS(SPI.Port.kMXP);
	}

	// public void stopFrontLeft() {
	// 	frontLeft.stopDriveMotor();
	// }

	// public void stopBackLeft() {
	// 	backLeft.stopDriveMotor();
	// }

	public void stopFrontRight() {
		frontRight.stopDriveMotor();
	}

	// public void stopBackRight() {
	// 	backRight.stopDriveMotor();
	// }

	// public void setFrontLeft(double speed) {
	// 	frontLeft.setDriveSpeed(speed);
	// }

	// public void setBackLeft(double speed) {
	// 	backLeft.setDriveSpeed(speed);
	// }

	public void setFrontRight(double speed) {
		frontRight.setDriveSpeed(speed);
	}

	// public void setBackRight(double speed) {
	// 	backRight.setDriveSpeed(speed);
	// }

	public void swerveDrive(double strafe, double forward, double omega, boolean deadStick) {
		double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
		double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);

		// Compute the constants used later for calculating speeds and angles
		double A = strafe - omegaL2;
		double B = strafe + omegaL2;
		double C = forward - omegaW2;
		double D = forward + omegaW2;

		// Compute the drive motor speeds
		double speedFL = speed(B, D);
		double speedBL = speed(A, D);
		double speedFR = speed(B, C);
		double speedBR = speed(A, C);

		// ... and angles for the steering motors
		// When drives are calibrated for zero position on encoders they are at 90
		// degrees
		// to the front of the robot. Subtract and add 90 degrees to steering
		// calculation to offset
		// for initial position/calibration of drives.

		double angleFL = angle(B, D) - Constants.FL_STEER_OFFSET;
		double angleBL = angle(A, D) + Constants.BL_STEER_OFFSET;
		double angleFR = angle(B, C) - Constants.FR_STEER_OFFSET;
		double angleBR = angle(A, C) + Constants.BR_STEER_OFFSET;
		// Compute the maximum speed so that we can scale all the speeds to the range
		// [0, 1]
		double maxSpeed = Collections.max(Arrays.asList(speedFL, speedBL, speedFR, speedBR, 1.0));

		if (deadStick){
			frontRight.setSteerSpeed(0);
			frontRight.setDriveSpeed(0);

		} else {
			// Set each swerve module, scaling the drive speeds by the maximum speed
			// frontLeft.setSwerve(angleFL, speedFL / maxSpeed);
			// backLeft.setSwerve(angleBL, speedBL / maxSpeed);
			frontRight.setSwerve(angleFR, speedFR / maxSpeed);
			// backRight.setSwerve(angleBR, speedBR / maxSpeed);
		}
	}

	private double speed(double val1, double val2) {
		return Math.hypot(val1, val2);
	}

	private double angle(double val1, double val2) {
		return Math.toDegrees(Math.atan2(val1, val2));
	}

	public static double[] getEncoderVal() {
		double[] values = new double[] {
			//frontLeft.getAnalogIn(),
			//backLeft.getAnalogIn(),
			frontRight.getAnalogIn()//,
			//backRight.getAnalogIn()
		};

		return values;
	}

	public double getNavHeading() {
        this.heading = navX.getFusedHeading();
        return heading;
     }

     public double getNavAngle() {
         this.angle = navX.getAngle();
         return angle;
     }


	public void initDefaultCommand() {
		//setDefaultCommand(new FieldCentricSwerveDrive());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

	}
}
