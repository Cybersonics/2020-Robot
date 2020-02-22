/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LauncherCommand extends CommandBase {

  // Member Variables

  Launcher _launcher;  
  Vision _vision;

  // End Member Variables

  // Constructors

  /**
   * constructor method
   *
   * @param launcher The launcher subsystem used by this command.
   * @param vision The vision subsystem used by this command.
   */
  public LauncherCommand(Launcher launcher, Vision vision) {
    _launcher = launcher;
    _vision = vision;

    addRequirements(launcher, vision);
  }

  // End Constructors

  // Public Methods

  public void fire() {
      _launcher.launcherStart();
  }  

  public void stop() {
      _launcher.launcherStop();
  }

  public void calculatedLaunch(double speed) {
    _launcher.calculatedLaunch(speed);
  }

  
  public void autonAngle() {
    _launcher.autonAngle();
}  

public void trenchAngle() {
    _launcher.trenchAngle();
}

public void bottomAngle() {
  _launcher.bottomAngle();
}


public void calculatedPivot(double speed) {
  _launcher.calculatedPivot(speed);
}

  // End Public Methods
}
