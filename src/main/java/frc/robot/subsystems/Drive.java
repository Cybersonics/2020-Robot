/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Arrays;
import java.util.Collections;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;


public class Drive extends SubsystemBase {

	private CommandScheduler scheduler;

	private static TalonSRX driveLeftFront;	
	private static TalonSRX driveLeftRear;
	private static TalonSRX driveRightFront;
	private static TalonSRX driveRightRear;
	private static TalonSRX steerLeftFront;
	private static TalonSRX steerLeftRear;
	private static TalonSRX steerRightFront;
	private static TalonSRX steerRightRear;

	public static final double WHEEL_BASE_LENGTH = 20; //28.0;
	public static final double WHEEL_BASE_WIDTH = 24; //22.0;
	public static final double ENCODER_COUNT_PER_ROTATION = 4096.0;

	public static final double WHEEL_DIAMETER = 4.0;
	//TODO: increase MAX_SPEED
	public static final double MAX_SPEED = 0.3; //Max speed is 0 to 1 
	public static final double STEER_DEGREES_PER_COUNT = 360.0 / 4096.0;
	public static final double DRIVE_INCHES_PER_COUNT = (WHEEL_DIAMETER * Math.PI) / (80.0 * 6.67);
	public static final double DEADZONE = 0.08;
	public static final double MAX_REVERSIBLE_SPEED_DIFFERENCE = 0.5 * MAX_SPEED;

	//private static final double STEER_P = 10.0, STEER_I = 0.02, STEER_D = 0.0;
	private static final double STEER_P = 5.0, STEER_I = 0.0, STEER_D = 0.0;
	private static final int STATUS_FRAME_PERIOD = 5;
	
  public Drive() {
	scheduler = CommandScheduler.getInstance();
	scheduler.registerSubsystem(new Drive());

    driveLeftFront = new TalonSRX(Constants.DRIVE_LEFT_FRONT_TALON);
	driveLeftFront.configFactoryDefault();
    driveLeftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    driveLeftFront.setInverted(false);
        
	driveLeftRear = new TalonSRX(Constants.DRIVE_LEFT_REAR_TALON);
	driveLeftRear.configFactoryDefault();
    driveLeftRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    driveLeftRear.setInverted(false);
  
	driveRightFront = new TalonSRX(Constants.DRIVE_RIGHT_FRONT_TALON);
	driveRightFront.configFactoryDefault();
    driveRightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    driveRightFront.setInverted(false);
  
	driveRightRear = new TalonSRX(Constants.DRIVE_RIGHT_REAR_TALON);
	driveRightRear.configFactoryDefault();
    driveRightRear.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    driveRightRear.setInverted(false);
  
	steerLeftFront = new TalonSRX(Constants.STEER_LEFT_FRONT_TALON);
	steerLeftFront.configFactoryDefault();
    steerLeftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    steerLeftFront.setInverted(true);
    steerLeftFront.config_kP(0, STEER_P, 0);
    steerLeftFront.config_kI(0, STEER_I, 0);
    steerLeftFront.config_kD(0, STEER_D, 0);
    steerLeftFront.config_IntegralZone(0, 100, 0);
    steerLeftFront.configAllowableClosedloopError(0, 5, 0);
    steerLeftFront.setNeutralMode(NeutralMode.Brake);
    //steerLeftFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

	steerLeftRear = new TalonSRX(Constants.STEER_LEFT_REAR_TALON);
	steerLeftRear.configFactoryDefault();
    steerLeftRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    steerLeftRear.setInverted(true);
    steerLeftRear.config_kP(0, STEER_P, 0);
    steerLeftRear.config_kI(0, STEER_I, 0);
    steerLeftRear.config_kD(0, STEER_D, 0);
    steerLeftRear.config_IntegralZone(0, 100, 0);
    steerLeftRear.configAllowableClosedloopError(0, 5, 0);
    steerLeftRear.setNeutralMode(NeutralMode.Brake);
    //steerLeftRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

	steerRightFront = new TalonSRX(Constants.STEER_RIGHT_FRONT_TALON);
	steerRightFront.configFactoryDefault();
    steerRightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    steerRightFront.setInverted(true);
    steerRightFront.config_kP(0, STEER_P, 0);
    steerRightFront.config_kI(0, STEER_I, 0);
    steerRightFront.config_kD(0, STEER_D, 0);
    steerRightFront.config_IntegralZone(0, 100, 0);
    steerRightFront.configAllowableClosedloopError(0, 5, 0);
    steerRightFront.setNeutralMode(NeutralMode.Brake);
    //steerRightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);

	steerRightRear = new TalonSRX(Constants.STEER_RIGHT_REAR_TALON);
	steerRightRear.configFactoryDefault();
    steerRightRear.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    steerRightRear.setInverted(true);
    steerRightRear.config_kP(0, STEER_P, 0);
    steerRightRear.config_kI(0, STEER_I, 0);
    steerRightRear.config_kD(0, STEER_D, 0);
    steerRightRear.config_IntegralZone(0, 100, 0);
    steerRightRear.configAllowableClosedloopError(0, 5, 0);
    steerRightRear.setNeutralMode(NeutralMode.Brake);
    //steerRightRear.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, STATUS_FRAME_PERIOD, 0);
	}

	public void swerveDrive(double strafe, double forward, double omega) {
    double omegaL2 = omega * (WHEEL_BASE_LENGTH / 2.0);
    double omegaW2 = omega * (WHEEL_BASE_WIDTH / 2.0);
    
    // Compute the constants used later for calculating speeds and angles
    double A = strafe - omegaL2;
    double B = strafe + omegaL2;
    double C = forward - omegaW2;
    double D = forward + omegaW2;
    
    // Compute the drive motor speeds
    double speedLF = speed(B, D);
    double speedLR = speed(A, D);
    double speedRF = speed(B, C);
    double speedRR = speed(A, C);
    
		// ... and angles for the steering motors 
		// When drives are calibrated for zero position on encoders they are at 90 degrees
		// to the front of the robot. Subtract and add 90 degrees to steering calculation to offset
		// for initial position/calibration of drives.

		double angleLF = angle(B, D) - 90;
    double angleLR = angle(A, D) + 90;
    double angleRF = angle(B, C) - 90;
    double angleRR = angle(A, C) + 90;
    // Compute the maximum speed so that we can scale all the speeds to the range [0, 1]
    double maxSpeed = Collections.max(Arrays.asList(speedLF, speedLR, speedRF, speedRR, 1.0));

    // Set each swerve module, scaling the drive speeds by the maximum speed
    setSwerveModule(steerLeftFront, driveLeftFront, angleLF, speedLF / maxSpeed);
    setSwerveModule(steerLeftRear, driveLeftRear, angleLR, speedLR / maxSpeed);
    setSwerveModule(steerRightFront, driveRightFront, angleRF, speedRF / maxSpeed);
		setSwerveModule(steerRightRear, driveRightRear, angleRR, speedRR / maxSpeed);
	}
	
	private double speed(double val1, double val2){
    return Math.hypot(val1, val2);
  }
  
  private double angle(double val1, double val2){
    return Math.toDegrees(Math.atan2(val1, val2));
  }
	

	private void setSwerveModule(TalonSRX steer, TalonSRX drive, double angle, double speed) {
    double currentPosition = steer.getSelectedSensorPosition(0);
    double currentAngle = (currentPosition * 360.0 / ENCODER_COUNT_PER_ROTATION) % 360.0;
    // The angle from the encoder is in the range [0, 360], but the swerve computations
    // return angles in the range [-180, 180], so transform the encoder angle to this range
    if (currentAngle > 180.0) {
      currentAngle -= 360.0;
    }
    // TODO: Properly invert the steering motors so this isn't necessary
    // This is because the steering encoders are inverted
    double targetAngle = -angle;
    double deltaDegrees = targetAngle - currentAngle;
    // If we need to turn more than 180 degrees, it's faster to turn in the opposite direction
    if (Math.abs(deltaDegrees) > 180.0) {
      deltaDegrees -= 360.0 * Math.signum(deltaDegrees);
    }
    // If we need to turn more than 90 degrees, we can reverse the wheel direction instead and
		// only rotate by the complement
		
		//if (Math.abs(speed) <= MAX_SPEED){
    if (Math.abs(deltaDegrees) > 90.0) {
      deltaDegrees -= 180.0 * Math.signum(deltaDegrees);
      speed = -speed;
    }
		//}
		

		double targetPosition = currentPosition + deltaDegrees * ENCODER_COUNT_PER_ROTATION / 360.0;
		steer.set(ControlMode.Position, targetPosition);
		drive.set(ControlMode.PercentOutput, speed);

	}

	//get Encoder values
	public double getDriveLFEncoder() {
		return driveLeftFront.getSelectedSensorPosition(0);
	}
	
	public double getDriveLREncoder() {
		return driveLeftRear.getSelectedSensorPosition(0);
	}
	
	public double getDriveRFEncoder() {
		return driveRightFront.getSelectedSensorPosition(0);
	}
	
	public double getDriveRREncoder() {
		return driveRightRear.getSelectedSensorPosition(0);
	}
	
	public double getSteerLFEncoder() {
		return steerLeftFront.getSelectedSensorPosition(0);
	}
	
	public double getSteerLREncoder() {
		return steerLeftRear.getSelectedSensorPosition(0);
	}
	
	public double getSteerRFEncoder() {
		return steerRightFront.getSelectedSensorPosition(0);
	}
	
	public double getSteerRREncoder() {
		return steerRightRear.getSelectedSensorPosition(0);
	}

	//setting motors
	public static void setDriveLeftFront(double speed){
		driveLeftFront.set(ControlMode.PercentOutput, speed);
	}

	public static void setDriveLeftRear(double speed){
		driveLeftRear.set(ControlMode.PercentOutput, speed);
	}
	
	public static void setDriveRightFront(double speed){
		driveRightFront.set(ControlMode.PercentOutput, speed);
	}
	
	public static void setDriveRightRear(double speed){
		driveRightRear.set(ControlMode.PercentOutput, speed);
	}
	
	public void setSteerLeftFront(double speed){
		steerLeftFront.set(ControlMode.PercentOutput, speed);
	}
	
	public void setSteerLeftRear(double speed){
		steerLeftRear.set(ControlMode.PercentOutput, speed);
	}
	
	public void setSteerRightFront(double speed){
		steerRightFront.set(ControlMode.PercentOutput, speed);
	}
	
	public void setSteerRightRear(double speed){
		steerRightRear.set(ControlMode.PercentOutput, speed);
  }
  
  public void encoderReset() {
		
		driveLeftFront.setSelectedSensorPosition(0, 0, 0);
    driveRightFront.setSelectedSensorPosition(0, 0, 0);
    driveLeftRear.setSelectedSensorPosition(0, 0, 0);
    driveRightRear.setSelectedSensorPosition(0, 0, 0);
  }
	
	public void initDefaultCommand() {
  }
  

  @Override
  public void periodic() {
	// This method will be called once per scheduler run
  }
}
