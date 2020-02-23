/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.frcteam2910.common.math.Rotation2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ResetGyro extends CommandBase {

  private Rotation2 rotation;
  private DrivetrainSubsystem _drive;
  private Navx _navx;
  /**
   * Creates a new ResetGyro.
   */
  public ResetGyro(Rotation2 rotation) {
    this.rotation = rotation;
    this._drive = DrivetrainSubsystem.getInstance();
    addRequirements(this._drive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drive.resetGyroAngle(this.rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
