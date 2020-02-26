/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class Feed extends CommandBase {

  private Indexer _indexer;
  private Intake _intake;
  private Launcher _launcher;
  private Timer _timer;
  private int duration;
  private int delay;
  /**
   * Creates a new Feed.
   */
  public Feed(Indexer indexer, Intake intake, int durationInSeconds, int delayInSeconds, Launcher launcher) {
    this._indexer = indexer;
    this._intake = intake;
    this.duration = durationInSeconds;
    this.delay = delayInSeconds;
    this._launcher = launcher;
    addRequirements(_indexer, _intake, _launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _timer = new Timer();
    _timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      _indexer.forward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _indexer.stop();
    _launcher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _timer.hasElapsed(this.duration);
  }
}