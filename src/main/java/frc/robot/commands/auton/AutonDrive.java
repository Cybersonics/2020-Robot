/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import net.bancino.robotics.swerveio.SwerveDrive;

public class AutonDrive extends CommandBase {

  private SwerveDrive swerve;
  private double distance;
  private double duration;
  private Timer _timer;
  /**
   * Creates a new Drive.
   */
  public AutonDrive(SwerveDrive swerve, double distance, double duration) {
    this.swerve = swerve;
    this.distance = distance;
    this.duration = duration;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this._timer = new Timer();
    _timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds vector = new ChassisSpeeds(distance / (duration/1000),0,  0);
    swerve.drive(vector);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _timer.hasElapsed((distance*1000)/1000);
  }
}
