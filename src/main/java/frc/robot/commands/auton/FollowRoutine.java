package frc.robot.commands.auton;

import java.util.function.Supplier;

import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class FollowRoutine extends CommandBase {
  private final Supplier<Trajectory> trajectorySupplier;

  private Trajectory trajectory;

  public FollowRoutine(Trajectory trajectory) {
    this(() -> trajectory);
  }

  public FollowRoutine(Supplier<Trajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;

        addRequirements(DrivetrainSubsystem.getInstance());
        this.runsWhenDisabled();
    }

    @Override
  public void initialize() {
    trajectory = trajectorySupplier.get();
    // DrivetrainSubsystem.getInstance().updateKinematics();
    DrivetrainSubsystem.getInstance().getFollower().follow(trajectory);
  }

    @Override
    public void end(boolean intterrupted) {
        // DrivetrainSubsystem.getInstance().setSnapRotation(trajectory.calculate(trajectory.getDuration()).getPathState().getRotation().toDegrees());
      if(intterrupted) {
        DrivetrainSubsystem.getInstance().getFollower().cancel();
      }
        // new Thread(() -> {
        //     Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
        //     Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        //     Timer.delay(0.5);
        //     Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        //     Robot.getOi().primaryController.getRawJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        // }).start();
    }

    @Override
    public boolean isFinished() {
        // Only finish when the trajectory is completed
        return DrivetrainSubsystem.getInstance().getFollower().getCurrentTrajectory().isEmpty();
    }
}