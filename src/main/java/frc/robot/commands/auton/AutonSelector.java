/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutonSelector {

  private static SendableChooser<Side> sideChooser;
  private static SendableChooser<Orientation> orientationChooser;

  private AutonRoutines _routines;

  static {
    final ShuffleboardTab autonTab = Shuffleboard.getTab("Autonomous settings");

    sideChooser = new SendableChooser<>();
    
    sideChooser.addOption("Left", Side.Left);
    sideChooser.setDefaultOption("Center", Side.Center);
    sideChooser.addOption("Right", Side.Right);
    sideChooser.addOption("Nothing", null);
    autonTab.add("Starting Side", sideChooser);

    orientationChooser = new SendableChooser<>();
    orientationChooser.setDefaultOption("Forward", Orientation.Forward);
    orientationChooser.addOption("Backwards", Orientation.Backwards);
    orientationChooser.addOption("Left", Orientation.Left);
    orientationChooser.addOption("Right", Orientation.Right);
    autonTab.add("Starting Orientation", orientationChooser);
  }

  public AutonSelector(AutonRoutines routines) {
    this._routines = routines;
  }

  public Command getCommand() {
    final Orientation startingOrientation = orientationChooser.getSelected();
    final Side startingSide = sideChooser.getSelected();

    Command group;
    if(startingSide == Side.Left) {
      group = _routines.getRotateFireMove(startingOrientation, startingSide);
    } else if (startingSide == Side.Right) {
      group = _routines.getRotateFireMove(startingOrientation, startingSide);
    } else {
      group = _routines.getRotateFireMove(startingOrientation, startingSide);
    }
       
    return group;

  }
}
