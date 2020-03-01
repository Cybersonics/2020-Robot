package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechIntake;

public class MechIntakeCommand  extends CommandBase {

  // Member Variables
  MechIntake _intake;
  DoubleSupplier _direction;

  public MechIntakeCommand(MechIntake intake, DoubleSupplier direction) {
    _intake = intake;
    _direction = direction;

    addRequirements(intake);
  }

    // public void forward() {
    //     _intake.forward();
    // }

    // public void reverse() {
    //     _intake.reverse();
    // }

    // public void stop() {
    //     _intake.stop();
    // }
}
