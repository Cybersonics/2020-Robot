/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.LinkedList;
import java.util.Queue;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.util.Side;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auton.AutonRoutines;
import frc.robot.commands.auton.Direction;
import frc.robot.commands.auton.FollowRoutine;
import frc.robot.commands.auton.MoveDistanceInDirection;
import frc.robot.commands.auton.RotateNintyDegreesCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonSelector {
  private final AutonRoutines routines;

  private static SendableChooser<Side> sideChooser;
  private static SendableChooser<Rotation2> orientationChooser;

  static {
    final ShuffleboardTab autonTab = Shuffleboard.getTab("Autonomous settings");

    sideChooser = new SendableChooser<>();
    sideChooser.addOption("Left", Side.LEFT);
    // sideChooser.addOption("Center", Side.Center);
    sideChooser.setDefaultOption("Right", Side.RIGHT);
    autonTab.add("Starting Side", sideChooser);

    orientationChooser = new SendableChooser<>();
    orientationChooser.setDefaultOption("Forward", Rotation2.ZERO);
    orientationChooser.addOption("Backwards", Rotation2.fromDegrees(180.0));
    orientationChooser.addOption("Left", Rotation2.fromDegrees(90.0));
    orientationChooser.addOption("Right", Rotation2.fromDegrees(270.0));
    autonTab.add("Starting Orientation", orientationChooser);
  }

  public AutonSelector(AutonRoutines routines) {
    this.routines = routines;
  }

  public Command getCommand() {
    final Rotation2 startingOrientation = orientationChooser.getSelected();
    final Side startingSide = sideChooser.getSelected();

      SequentialCommandGroup group = new SequentialCommandGroup(
        new InstantCommand(() -> {
          DrivetrainSubsystem.getInstance().resetGyroAngle(startingOrientation);
        })
      );

      // group.addCommands(
      //   new FollowRoutine(this.routines.getMoveBackTrajectory(startingSide))
      // );

      group.addCommands(
        new MoveDistanceInDirection(24.0, Direction.FORWARD)
      );
       
        return group;
    }
}