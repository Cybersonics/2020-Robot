package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.frcteam2910.common.math.Vector2;

public class RotateNintyDegreesCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier rotation;
    private final BooleanSupplier fieldCentric;

    public RotateNintyDegreesCommand(DoubleSupplier forward,
                        DoubleSupplier strafe,
                        DoubleSupplier rotation,
                        BooleanSupplier fieldCentric) {
        this.drivetrain = DrivetrainSubsystem.getInstance();
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.fieldCentric = fieldCentric;

        addRequirements(this.drivetrain);
    }

	@Override
    public void execute() {
        drivetrain.drive(    
            new Vector2(
                forward.getAsDouble(),
                strafe.getAsDouble()
            ),
                rotation.getAsDouble(),
                fieldCentric.getAsBoolean()            
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(Vector2.ZERO, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        return drivetrain.getPose().rotation.toDegrees() == 90.0;
    }
}