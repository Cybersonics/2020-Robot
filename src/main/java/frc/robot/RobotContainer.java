/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.robot.UpdateManager;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeIndexerControl;
import frc.robot.commands.Navx;
import frc.robot.commands.ShooterControl;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

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
  
  private final Intake _intake_subsystem = new Intake();
  private final Indexer _indexer_subsystem = new Indexer();
  private final Shooter _shooter_subsystem = new Shooter();
  private final DrivetrainSubsystem _drive_subsystem = new DrivetrainSubsystem();
  
    private final IntakeIndexerControl _intake_indexer_command = new IntakeIndexerControl(_intake_subsystem, _indexer_subsystem);
    private final ShooterControl _shooter_command = new ShooterControl(_shooter_subsystem);

  private final AutoCommand m_autoCommand = new AutoCommand();

  public static Joystick leftJoy;
  public static Joystick rightJoy;
  public XboxController xboxController;
  
  private final UpdateManager updateManager = new UpdateManager(_drive_subsystem);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    leftJoy = new Joystick(Constants.LEFT_JOYSTICK);
    rightJoy = new Joystick(Constants.RIGHT_JOYSTICK);
    xboxController = new XboxController(Constants.XBOX_CONTROLLER);

      _drive_subsystem.setDefaultCommand(
        new DriveCommand(
          _drive_subsystem,
          () -> leftJoy.getY(Hand.kLeft),
          () -> leftJoy.getX(Hand.kLeft),
          () -> rightJoy.getX(Hand.kRight),
          leftJoy.getTrigger()   
        )
      );

      updateManager.startLoop(5.0e-3);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(leftJoy, 7).whenPressed(() -> _drive_subsystem.resetGyroAngle(Rotation2.ZERO));

    // new JoystickButton(xboxController, 1).whenPressed(() -> _intake_indexer_command.indexerForward());
    // new JoystickButton(xboxController, 1).whenReleased(() -> _intake_indexer_command.indexerStop());
    // new JoystickButton(xboxController, 2).whenPressed(() -> _intake_indexer_command.indexerReverse());
    // new JoystickButton(xboxController, 2).whenReleased(() -> _intake_indexer_command.indexerStop());

    // new JoystickButton(xboxController, 3).whenPressed(() -> _intake_indexer_command.intakeForward());
    // new JoystickButton(xboxController, 3).whenReleased(() -> _intake_indexer_command.intakeStop());
    // new JoystickButton(xboxController, 4).whenPressed(() -> _intake_indexer_command.intakeReverse());
    // new JoystickButton(xboxController, 4).whenReleased(() -> _intake_indexer_command.intakeStop());

    // new JoystickButton(xboxController, 6).whenPressed(() -> _shooter_command.fire());
    // new JoystickButton(xboxController, 6).whenReleased(() -> _shooter_command.stop());
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
