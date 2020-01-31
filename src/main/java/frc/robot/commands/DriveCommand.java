package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    private final Drive drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;

    public DriveCommand(Drive drivetrain,
                        DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier rotation) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        addRequirements(drivetrain);
    }

	@Override
    public void execute() {
        drivetrain.swerveDrive(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                rotation.getAsDouble()                
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.swerveDrive(0.0, 0.0, 0.0);
    }
}