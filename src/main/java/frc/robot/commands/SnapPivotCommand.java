/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class SnapPivotCommand extends CommandBase {

  private Launcher _launcher;
  private double _pivotSetpoint;

  /**
   * Creates a new PivotCommand.
   */
  public SnapPivotCommand(Launcher launcher, double pivotSetpoint) {
    this._launcher = launcher;
    this._pivotSetpoint = pivotSetpoint;
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this._pivotSetpoint > -700 || this._pivotSetpoint < -888) {
      System.out.println("[PivotCommand] Ending command bad setpoint passed");
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("[PivotCommand] TargetSetPoint: " + _pivotSetpoint + " Current Angle: " +  Launcher.getPivotAngle());
    this._launcher.calculatedPivot(this._pivotSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("[PivotCommand] interrupted due to bad setpoint value");
    }
    this._launcher.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this._pivotSetpoint - Launcher.getPivotAngle()) < 3;
  }
}