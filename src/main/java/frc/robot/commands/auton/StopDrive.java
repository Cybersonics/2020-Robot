/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class StopDrive extends InstantCommand {
  public StopDrive() {
		addRequirements(DrivetrainSubsystem.getInstance());
	}
	
	@Override
	public void initialize() {
		DrivetrainSubsystem.getInstance().drive(Vector2.ZERO, 0.0, true);
	}
}
