/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Location;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.ShooterControl;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.auton.Alderaan;
import frc.robot.commands.auton.Drive;
import frc.robot.commands.auton.Rotate;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;
import net.bancino.robotics.swerveio.command.RunnableCommand;
import net.bancino.robotics.swerveio.gyro.NavXGyro;

/**
 * Add your docs here.
 */
public class AutonRoutines {

    private DriveTrain _drive;
    private Launcher _launcher;
    private Vision _vision;
    private Intake _intake;
    private Indexer _indexer;

    public AutonRoutines(DriveTrain drive, Launcher launcher, Vision vision, Intake intake, Indexer indexer) {
        this._drive = drive;
        this._launcher = launcher;
        this._vision = vision;
        this._indexer = indexer;
        this._intake = intake;
    }

    public Command getRotateFireMove(Orientation orientaition, Side side) {
        Command group; 

        if (Side.Left == side) {
            group = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    _drive.getGyro().zero();
                }), 
                new PivotCommand(_launcher, -850),
                new ParallelCommandGroup(
                    new Rotate(_drive, (NavXGyro) _drive.getGyro(), 61, 3500, _launcher, true),
                    new PivotCommand(_launcher, Location.Auton+10)
                ),
                new Alderaan(_launcher, _vision, _intake, _indexer),
                new Drive(_drive, .3, .5)
            );
        } else if ( Side.Right == side) {
            group = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    _drive.getGyro().zero();
                }), 
                new ParallelCommandGroup(new Rotate(_drive, (NavXGyro) _drive.getGyro(), 100, 3500, _launcher, true),
                    new PivotCommand(_launcher, Location.Auton)
                ),
                new Alderaan(_launcher, _vision, _intake, _indexer),
                new Rotate(_drive, (NavXGyro) _drive.getGyro(), 58, 3000, _launcher, false),
                new Drive(_drive, .5, 1)
            );
        } else if (Side.Center == side) {
            group = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    _drive.getGyro().zero();
                }), 
                new ParallelCommandGroup(
                    new Rotate(_drive, (NavXGyro) _drive.getGyro(), 90, 3000, _launcher, true),
                    new PivotCommand(_launcher, Location.Auton)
                ),
                new Alderaan(_launcher, _vision, _intake, _indexer),
                new Rotate(_drive, (NavXGyro) _drive.getGyro(), 90, 3000, _launcher, false),
                new Drive(_drive, .5, 1)
            );
        } else {
           group = new InstantCommand();
        }

        return group;
    }

}