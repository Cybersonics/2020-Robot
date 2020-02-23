/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.util.Side;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Add your docs here.
 */
public class AutonRoutines {

    private final Trajectory moveBackTrajectory;
    
    final ShuffleboardTab autonTab = Shuffleboard.getTab("Trajectory");
    String selectedAuton;
    
    public AutonRoutines() {
       // First we have to generate our path. We will use the SplinePathBuilder to generate a path using splines.
       Path path = new SplinePathBuilder(Vector2.ZERO, Rotation2.ZERO, Rotation2.ZERO)
            // When using hermite splines we must specify a position and a heading. We can also optionally specify
            // a rotation.
            .hermite(new Vector2(0.0, 24.0), Rotation2.ZERO)
            // Once we've added all the splines we can then build the path.
            .build();

        // Once we have our path we need to then specify some constraints for our trajectory.
        TrajectoryConstraint[] constraints = {
            // Lets specify a maximum acceleration of 10.0 units/s^2
            new MaxAccelerationConstraint(10.0),
            // And lets have a maximum velocity of 12.0 units/s
            new MaxVelocityConstraint(12.0)
        };

        // Now that we have both our path and our constraints we can create a trajectory.
        // When creating a trajectory we pass in our path and our constraints.
        // We also have to pass in a third parameter called sample distance. This sample distance
        // determines how often the trajectory makes sure that the velocity and acceleration are within
        // the limits determined by the constraints. Smaller values will create a smoother and more accurate path
        // but they will take much longer to generate.
        moveBackTrajectory = new Trajectory(path, constraints, 1.0e-2);
    }

    public Trajectory getMoveBackTrajectory(Side side) {
            return moveBackTrajectory;
    }

    public void printCSVTrajectory(Trajectory trajectory) {
            int samples = (int) Math.ceil(trajectory.getDuration() / 1.0e-2);
            ShuffleboardLayout frontLeftModuleContainer = autonTab.getLayout("forwardTrajectory", BuiltInLayouts.kGrid)
                                .withPosition(1, 0).withSize(2, 3);

            for (int i = 0; i <= samples; i++) {
                // Calculate the trajectory state at the given time
                Trajectory.State state = trajectory.calculate(i * 1.0e-2);

                // Write the trajectory state to the Console
                frontLeftModuleContainer.add(i + "time", (i * 1.0e-2));
                frontLeftModuleContainer.add(i + "distance", state.getPathState().getDistance());
                frontLeftModuleContainer.add(i + "x", state.getPathState().getPosition().x);
                frontLeftModuleContainer.add(i + "y", state.getPathState().getPosition().y);
                frontLeftModuleContainer.add(i + "heading", state.getPathState().getHeading().toDegrees());
                frontLeftModuleContainer.add(i + "rotation", state.getPathState().getRotation().toDegrees());
                frontLeftModuleContainer.add(i + "curvature", state.getPathState().getCurvature());
                frontLeftModuleContainer.add(i + "velocity", state.getVelocity());
                frontLeftModuleContainer.add(i + "acceleration", state.getAcceleration());

            }
    }
}