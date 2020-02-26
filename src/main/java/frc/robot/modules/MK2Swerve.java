package frc.robot.modules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import net.bancino.robotics.swerveio.encoder.AnalogEncoder;
import net.bancino.robotics.swerveio.encoder.AbstractEncoder;
import net.bancino.robotics.swerveio.encoder.SparkMaxEncoder;
import net.bancino.robotics.swerveio.pid.AbstractPIDController;

import net.bancino.robotics.swerveio.geometry.Length;
import net.bancino.robotics.swerveio.geometry.Unit;
import net.bancino.robotics.swerveio.module.GenericSwerveModule;

/**
 * A swerve module implementation that uses RevRobotics Neo motors and Spark Max
 * motor controllers. This was designed for Swerve Drive Specialties' MK2
 * Module.
 * 
 * @author Jeremy Reed
 * @version 1.0.0
 * @since 1.0.0
 */
public class MK2Swerve extends GenericSwerveModule {

    private static final double GEAR_RATIO = 9.62;
    private static final double DEFAULT_MAX_DRIVE_MOTOR_RPMS = 5680;
    private static final Length WHEEL_DIAMETER = new Length(4, Unit.INCHES);

    private CANSparkMax driveMotor, pivotMotor;

    /**
     * The swerve module is constructed to allow the pivot motor to coast, this
     * allows for adjustments, but as soon as the module is driven, it switches to
     * brake mode to prevent outside modifications.
     */
    private boolean setPivotIdleMode = false;

    /**
     * Create a new swerve module composed of Neo brushless motors, this uses spark
     * max motor controllers.
     * 
     * @param driveCanId The CAN ID of the drive motor for this module.
     * @param pivotCanId The CAN ID of the pivot motor for this module.
     * @param encoder    The encoder to use as the pivot encoder.
     */
    public MK2Swerve(int driveCanId, int pivotCanId, AbstractEncoder encoder) {
        super(new CANSparkMax(driveCanId, MotorType.kBrushless), new CANSparkMax(pivotCanId, MotorType.kBrushless),
                new SparkMaxEncoder(), encoder, GEAR_RATIO, DEFAULT_MAX_DRIVE_MOTOR_RPMS, WHEEL_DIAMETER);
        driveMotor = (CANSparkMax) getDriveMotor();
        pivotMotor = (CANSparkMax) getPivotMotor();
        pivotMotor.setIdleMode(IdleMode.kCoast);
        driveMotor.setIdleMode(IdleMode.kBrake);
        ((SparkMaxEncoder) getDriveEncoder()).setController(driveMotor);
        
        if (encoder instanceof SparkMaxEncoder) {
            try {
                ((SparkMaxEncoder) getPivotEncoder()).setController(pivotMotor);
            } catch (UnsupportedOperationException e) {
                /* The passed encoder must already be monitoring a controller.
                 * Do nothing and hope the user knows what they're doing.
                 */
            }
        }

        drivePid.setOutputLimits(-1, 1);

        /* Set sensible defaults for this module. */
        AbstractPIDController pivotPid = getPivotPIDController();
        //pivotPid.setP(0.004);
        pivotPid.setI(0);
        pivotPid.setD(0);
    }

    /**
     * Create a new swerve module composed of Neo brushless motors, this uses spark
     * max motor controllers.
     * 
     * @param driveCanId        The CAN ID of the drive motor for this module.
     * @param pivotCanId        The CAN ID of the pivot motor for this module.
     * @param analogEncoderPort The port on the roboRIO that the encoder to use as
     *                          the pivot encoder is on.
     */
    public MK2Swerve(int driveCanId, int pivotCanId, int analogEncoderPort) {
        this(driveCanId, pivotCanId, new AnalogEncoder(analogEncoderPort));
    }

    @Override
    public void setPivotMotorSpeed(double speed) {
        if (!setPivotIdleMode) {
            pivotMotor.setIdleMode(IdleMode.kBrake);
            setPivotIdleMode = true;
        }
        super.setPivotMotorSpeed(speed);
    }

    @Override
    public void setDriveReference(double ref) {
        driveMotor.getPIDController().setReference(ref, ControlType.kPosition);
    }

    @Override
    public void setDriveMotorSpeed(double ref) {
        driveMotor.set(ref);
    }
}