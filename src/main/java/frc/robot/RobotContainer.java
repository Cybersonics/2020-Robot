/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.function.DoubleSupplier;

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
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.Navx;
import frc.robot.commands.LauncherCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // #region Member Variables
  
  // The robot's subsystems and commands are defined here...
  // #region Subsystems
  private final DrivetrainSubsystem _drive = new DrivetrainSubsystem();
  private final Indexer _indexer = new Indexer();
  private final Intake _intake = new Intake();
  private final Launcher _launcher = new Launcher();
  private final Vision _vision = new Vision();
  // #endregion End Subsystems

  // #region Commands
  private final IntakeCommand _intakeCommand = new IntakeCommand(_intake);
  private final IndexerCommand _indexerCommand = new IndexerCommand(_indexer);
  private final LauncherCommand _launcherCommand = new LauncherCommand(_launcher, _vision);

  private final AutoCommand m_autoCommand = new AutoCommand();

  // #endregion End Commands

  // #region Controllers

  public static Joystick leftJoy;
  public static Joystick rightJoy;
  public XboxController xboxController;
  
  // #endregion End Controllers

  public static final double DEADZONE = 0.1;
	public static final double OMEGA_SCALE = 1.0 / 30.0;
	private final double leftPow = 1.0;
	private final double rightPow = 1.0;
  private final UpdateManager updateManager = new UpdateManager(_drive);
  
  // #endregion End Member Variables
  
  // #region Constructors
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    leftJoy = new Joystick(Constants.LEFT_JOYSTICK);
    rightJoy = new Joystick(Constants.RIGHT_JOYSTICK);
    xboxController = new XboxController(Constants.XBOX_CONTROLLER);

    _drive.setDefaultCommand(
      new DriveCommand(
        _drive,
        () -> deadzoneLeftController(leftJoy.getY(Hand.kLeft)),
        () -> deadzoneLeftController(leftJoy.getX(Hand.kLeft)),
        () -> deadzoneRightController(rightJoy.getX(Hand.kRight)) * .1,
        () -> leftJoy.getTrigger()   
      )
    );

    _indexer.setDefaultCommand(new IndexerCommand(
      _indexer,
      () -> xboxController.getY(Hand.kRight)
    ));

    _intake.setDefaultCommand(new IntakeCommand(
      _intake, 
      () -> xboxController.getY(Hand.kLeft)
    ));

    updateManager.startLoop(5.0e-3);

    configureButtonBindings();
  }

  // #endregion End Constructors

  // #region Private Methods
  private double deadzoneLeftController(double controllerValue) {
    double power = Math.pow(Math.abs(controllerValue), leftPow) * Math.signum(controllerValue);

		// Add a small deadzone on the joysticks
		if (Math.abs(power) < Math.pow(DEADZONE, leftPow)) {
      power = 0.0;
    }
    
    return power;
  }

  private double deadzoneRightController(double controllerValue) {
    double power = Math.pow(Math.abs(controllerValue), rightPow) * Math.signum(controllerValue) * OMEGA_SCALE;

		// Add a small deadzone on the joysticks
		if (Math.abs(power) < Math.pow(DEADZONE, rightPow) * OMEGA_SCALE)
			power = 0.0;
    
    return power;
  }



  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(leftJoy, 7).whenPressed(() -> _drive.resetGyroAngle(Rotation2.ZERO));

    new JoystickButton(xboxController, 6).whenPressed(() -> _launcherCommand.fire());
    new JoystickButton(xboxController, 6).whenReleased(() -> _launcherCommand.stop());
  }
  
  // #endregion End Private Methods

  // #region Public Methods

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public  AutoCommand getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  // #endregion End Public Commands
}
