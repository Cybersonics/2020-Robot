/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class AdjustPivotCommand extends CommandBase {

  private Launcher _launcher;
  private double _pivotPoint;
  private double _adjustment;
  private boolean _isProcessing = false;



  /**
   * Creates a new PivotCommand.
   */
  public AdjustPivotCommand(Launcher launcher, double adjustment) {
    this._launcher = launcher;
    this._adjustment = adjustment;
    
    addRequirements(launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (this._pivotPoint > -700 || this._pivotPoint < -888) {
      System.out.println("[PivotCommand] Ending command bad setpoint passed");
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!_isProcessing) {
        this._isProcessing = true;
        this._pivotPoint = ((Launcher.getPivotAngle() + this._adjustment) > -700 || (Launcher.getPivotAngle() + this._adjustment) < -888) ? Launcher.getPivotAngle() : (Launcher.getPivotAngle() + this._adjustment);
        System.out.println("[PivotCommand] Adjustment: " + this._adjustment +" TargetSetPoint: " + _pivotPoint + " Current SetPoint: " +  Launcher.getPivotAngle());
    }
    this._launcher.calculatedPivot(this._pivotPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("[PivotCommand] interrupted due to bad setpoint value");
    }
    this._launcher.stopPivot();
    System.out.println("[PivotCommand] Ended");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (Math.abs(this._pivotPoint - Launcher.getPivotAngle()) < 3) {
        this._isProcessing = false;
        return true;
      } else {
          return false;
      }
  }
}
