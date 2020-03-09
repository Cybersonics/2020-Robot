package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechIntake;

public class MechIntakeCommand extends CommandBase {

  // Member Variables
  MechIntake _intake;
  DoubleSupplier _direction;

  public MechIntakeCommand(MechIntake intake) {
    _intake = intake;

    addRequirements(intake);
  }

  public MechIntakeCommand(MechIntake intake, DoubleSupplier direction) {
    _intake = intake;
    _direction = direction;

    addRequirements(intake);
  }

    public void forward() {
        _intake.forward();
    }

    public void reverse() {
        _intake.reverse();
    }

    public void stop() {
        _intake.stop();
    }

    
  @Override
  public void execute() {
    if (_direction != null) {
      _intake.manualControl(_direction.getAsDouble());
    }
  }

  @Override
  public void end(boolean interrupted) {
    _intake.stop();
  }
}
