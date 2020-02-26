package frc.robot.commands.auton;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.command.PathweaverSwerveDrive;
import net.bancino.robotics.swerveio.gyro.NavXGyro;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PathWeaverAuton extends SequentialCommandGroup {
  /**
   * Creates a new HallAuton.
   */
  public PathWeaverAuton(SwerveDrive swerve, String pathweaverJson) throws IOException {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      // new Rotate(swerve, (NavXGyro) swerve.getGyro(), 90, 2500)
      new PathweaverSwerveDrive(swerve, pathweaverJson)
    );
  }
}