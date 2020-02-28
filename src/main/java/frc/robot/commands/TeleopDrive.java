package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.geometry.SwerveVector;

/**
 * A simple command for driving a swerve drive. 
 *
 * @author Jordan Bancino
 * @version 2.1.0
 * @since 2.0.0
 */
public class TeleopDrive extends CommandBase {

    private SwerveDrive swerve;
    private Joystick driveJoystick;
    private Joystick rotationJoystick;
    private Joystick.AxisType fwdAxis, strAxis, rcwAxis;

    private double deadband, throttle;

    /**
     * Create a new swerve drive joystick command.
     *
     * @param swerve The swerve drive to drive.
     * @param driveJoystick The joystick to read from.
     * @param rotationJoystick The joystick to read from.
     * @param fwdAxis The axis to read from that will provide values for the Y movement of the swerve drive.
     * @param strAxis The axis to read from that will provide values for the X movement of the swerve drive.
     * @param rcwAxis The axis to read from that will provide values for the angular movement of the swerve drive.
     */
    public TeleopDrive(SwerveDrive swerve, Joystick driveJoystick, Joystick.AxisType fwdAxis, Joystick.AxisType strAxis, Joystick rotationJoystick, Joystick.AxisType rcwAxis) {
        if (swerve != null) {
            this.swerve = swerve;
        } else {
            throw new IllegalArgumentException("Swerve Drive cannot be null.");
        }
        if (driveJoystick != null) {
            this.driveJoystick = driveJoystick;
            this.rotationJoystick = rotationJoystick;
            this.fwdAxis = fwdAxis;
            this.strAxis = strAxis;
            this.rcwAxis = rcwAxis;
        } else {
            throw new IllegalArgumentException("Xbox controller cannot be null.");
        }

        addRequirements(swerve);
        setDeadband(0.2);
        setThrottle(0.9);
    }

    /**
     * Set a deadband on the joystick. This is a little bit of range around the zero mark that
     * does absolutely nothing. This is helpful for joysticks that are overly sensitive or
     * don't always snap read zero in the neutral position.
     * @param deadband The deadband to set, between 0 and 1.
     */
    public void setDeadband(double deadband) {
      this.deadband = deadband;
    }

    /**
     * Set a throttle on the joystick. This is helpful for limiting the top speed of the swerve drive.
     * @param throttle The throttle to set. This is the maximum speed the joystick should output. So, to
     * throttle this command at 50% power, you'd put in 0.5 for the throttle.
     */
    public void setThrottle(double throttle) {
      this.throttle = throttle;
    }

    @Override
    public void execute() {
        double fwd = throttle(deadband(driveJoystick.getRawAxis(fwdAxis.value)));
        double str = -throttle(deadband(driveJoystick.getRawAxis(strAxis.value)));
        double rcw = -throttle(deadband(rotationJoystick.getRawAxis(rcwAxis.value)));
        SwerveVector joystickVector = new SwerveVector(fwd, str, rcw);
        swerve.drive(joystickVector);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

  private double throttle(double raw) {
    return raw * throttle;
  }

  /**
   * Calculate a deadband
   * 
   * @param raw The input on the joystick to mod
   * @return The result of the mod.
   */
  private double deadband(double raw) {
    /* This will be our result */
    double mod;
    /* Compute the deadband mod */
    if (raw < 0.0d) {
      if (raw <= -deadband) {
        mod = (raw + deadband) / (1 - deadband);
      } else {
        mod = 0.0d;
      }
    } else {
      if (raw >= deadband) {
        mod = (raw - deadband) / (1 - deadband);
      } else {
        mod = 0.0d;
      }
    }
    /* Return the result. */
    return mod;
  }
}