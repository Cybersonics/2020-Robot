package frc.robot.commands.auton;

import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveDistanceInDirection extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final double distance;
    private final Direction direction;

    private double forward;
    private double strafe;

    private double[] startingDistances;

    public MoveDistanceInDirection(double distance, Direction direction) {
        this.drivetrain = DrivetrainSubsystem.getInstance();
        this.distance = distance;
        this.direction = direction;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        startingDistances = new double[drivetrain.getSwerveModules().length];
        for (int i = 0; i < startingDistances.length; i++) {
            startingDistances[i] = drivetrain.getSwerveModules()[i].getCurrentDistance();
        }
    }

    @Override
    public void execute() {
        switch(this.direction) {
            case FORWARD:
                this.forward = 0.5;
            case LEFT:
                this.strafe = -0.5;
            case RIGHT:
                this.strafe = 0.5;
            case BACK:
                this.forward = -0.5;
            default:
                this.forward = 0.0;
                this.strafe = 0.0;
            
        }
        drivetrain.drive(new Vector2(this.forward, this.strafe), 0.0, true);
    }

    @Override
    public void end(boolean interrupted) {
        this.forward = 0.0;
        this.strafe = 0.0;
    }

    @Override
    public boolean isFinished() {
        double avgDistance = 0.0;
        int moduleCount = 0;
        for (int i = 0; i < startingDistances.length; i++) {
            SwerveModule module = drivetrain.getSwerveModules()[i];
            if (module.getModulePosition().y > 0.0) {
                double delta = Math.abs(module.getCurrentDistance() - startingDistances[i]);

                avgDistance += delta;
                moduleCount++;
            }
        }

        if (moduleCount == 0) {
            return false;
        } else {
            return avgDistance / moduleCount > distance;
        }
    }
}