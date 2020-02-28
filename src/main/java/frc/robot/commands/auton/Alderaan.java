/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Launcher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Alderaan extends ParallelCommandGroup {
  /**
   * Creates a new alderaan.
   */
  public Alderaan(Launcher launcher, Vision vision, Intake intake, Indexer indexer) {
    // Add your commands in the super() call, e.g.
    super(
      // new Launch(launcher, vision, 6),
      new Feed(indexer, intake, 3, 0, launcher)
    );
  }
}
