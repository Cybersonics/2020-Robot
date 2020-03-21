/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FieldCentricSwerveDrive;
import frc.robot.commands.IntakeIndexerControl;
import frc.robot.commands.ShooterControl;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
// import frc.robot.commands.FieldCentricSwerveDrive;
import frc.robot.subsystems.Navx;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /*
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  */

  //private final IntakeIndexerControl intakeCommand = new IntakeIndexerControl();
  //private final ShooterControl shooterCommand = new ShooterControl();

  //private final Intake INTAKE_SUBSYSTEM = new Intake();
  //private final Indexer INDEXER_SUBSYSTEM = new Indexer();
  //private final Shooter SHOOTER_SUBSYSTEM = new Shooter();
  public static Drive driveSub = new Drive();
 
  private final AutoCommand m_autoCommand = new AutoCommand();

  public static Joystick leftJoy;
  public static Joystick rightJoy;
  public XboxController controller;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    leftJoy = new Joystick(Constants.LEFT_JOYSTICK);
    rightJoy = new Joystick(Constants.RIGHT_JOYSTICK);
    controller = new XboxController(Constants.XBOX_CONTROLLER);

    CommandScheduler.getInstance()
    .setDefaultCommand(
      driveSub,
      new FieldCentricSwerveDrive(
        driveSub,
        () -> leftJoy.getY(),
        () -> leftJoy.getX(),
        () -> rightJoy.getX(),
        leftJoy.getTrigger()
      )
    );
    // driveSub
    //   .setDefaultCommand( 
    //     FieldCentricSwerveDrive.getInstance() 
    //   );

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  //new JoystickButton(leftJoy, 7).whenPressed(() -> Navx.getInstance().getFuzedHeading());
    new JoystickButton(leftJoy, 7).whenPressed(() -> driveSub.getNavHeading());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public AutoCommand getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

}
