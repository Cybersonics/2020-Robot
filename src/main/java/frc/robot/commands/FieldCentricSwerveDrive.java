/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/**
 * An example command that uses an example subsystem.
 */
public class FieldCentricSwerveDrive extends CommandBase {
	
	public static final double OMEGA_SCALE = 1.0 / 30.0;
	public static final double DEADZONE = 0.05;

	private double originHeading = 0.0;
	private double originCorr = 0;
  	private final double leftPow = 1.0;
	private final double rightPow = 1.0;
	private final Drive _drive = new Drive();

	public FieldCentricSwerveDrive() {
	}

	// Called when the command is initially scheduled.

	@Override
	public void execute() {

		final double originOffset = 360 - originHeading;
		originCorr = Constants.navX.getFusedHeading() + originOffset;

		double strafe = Math.pow(Math.abs(RobotContainer.leftJoy.getX()), leftPow)
				* Math.signum(RobotContainer.leftJoy.getX());
		double forward = Math.pow(Math.abs(RobotContainer.leftJoy.getY()), leftPow)
				* -Math.signum(RobotContainer.leftJoy.getY());
		double omega = Math.pow(Math.abs(RobotContainer.rightJoy.getX()), rightPow)
				* Math.signum(RobotContainer.rightJoy.getX()) * OMEGA_SCALE;

		// Add a small deadzone on the joysticks
		if (Math.abs(strafe) < Math.pow(DEADZONE, leftPow))
			strafe = 0.0;
		if (Math.abs(forward) < Math.pow(DEADZONE, leftPow))
			forward = 0.0;
		if (Math.abs(omega) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE)
			omega = 0.0;

		// If all of the joysticks are in the deadzone, don't update the motors
		// This makes side-to-side strafing much smoother
		if (strafe == 0.0 && forward == 0.0 && omega == 0.0) {
			Drive.setDriveLeftFront(0.0);
			Drive.setDriveLeftRear(0.0);
			Drive.setDriveRightFront(0.0);
			Drive.setDriveRightRear(0.0);
			return;
		}

		if (!RobotContainer.leftJoy.getTrigger()) {
			// When the Left Joystick trigger is not pressed, The robot is in Field Centric
			// Mode.
			// The calculations correct the forward and strafe values for field centric
			// attitude.

			// Rotate the velocity vector from the joystick by the difference between our
			// current orientation and the current origin heading
			final double originCorrection = Math.toRadians(originHeading - Constants.navX.getFusedHeading());
			final double temp = forward * Math.cos(originCorrection) - strafe * Math.sin(originCorrection);
			strafe = strafe * Math.cos(originCorrection) + forward * Math.sin(originCorrection);
			forward = temp;
		}

		this._drive.swerveDrive(strafe, forward, omega);
	}


}