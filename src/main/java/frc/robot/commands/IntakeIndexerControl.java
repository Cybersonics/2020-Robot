/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/**
 * An example command that uses an example subsystem.
 */
public class IntakeIndexerControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  Intake _intake;
  Indexer _indexer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeIndexerControl(Intake intake, Indexer indexer) {
    // Creating new Intake object
    _intake = intake;
    
    // Creating new Indexer object
    _indexer = indexer;

    CommandScheduler.getInstance().requiring(intake);
    CommandScheduler.getInstance().requiring(indexer);
  }

public void intakeForward() {
  _intake.intakeRun();
}

public void intakeReverse() {
  _intake.intakeReverse();
}

public void intakeStop() {
  _intake.intakeStop();
}

public void indexerForward() {
  _indexer.forward();
}

public void indexerReverse() {
  _indexer.reverse();
}

public void indexerStop() {
  _indexer.StopMotor();
}



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // // Intake Runs In
    // if(Robot.xBoxController.getPOV() == 90) 
    // {
    //   intake.intakeStop();
    //   intake.intakeRun();
    //   indexer.RunMotor();
    // } 
    // // Intake Goes Backward
    // else if(Robot.xBoxController.getPOV() == 270) 
    // {
    //   intake.intakeStop();
    //   intake.intakeReverse();
    //   indexer.StopMotor();
    // }
    // // Intake Shuts off
    // else if (Robot.xBoxController.getPOV() == 180) 
    // {
    //   intake.intakeStop();
    //   indexer.StopMotor();
    // } 
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