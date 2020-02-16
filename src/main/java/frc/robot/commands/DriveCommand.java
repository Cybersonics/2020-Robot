package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;
    private final boolean fieldCentric;

    public DriveCommand(DrivetrainSubsystem drivetrain,
                        DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier rotation,
                        boolean fieldCentric) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.fieldCentric = fieldCentric;

        addRequirements(drivetrain);
    }

	@Override
    public void execute() {
        drivetrain.drive(    
            new Vector2(
                forward.getAsDouble(),
                strafe.getAsDouble()
            ),
                rotation.getAsDouble(),
                fieldCentric                
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(Vector2.ZERO, 0.0, false);
    }
}