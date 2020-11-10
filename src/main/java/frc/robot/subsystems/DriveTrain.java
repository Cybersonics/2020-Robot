/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import frc.robot.Constants;
import net.bancino.robotics.swerveio.SwerveDrive;
import net.bancino.robotics.swerveio.SwerveFlag;
import net.bancino.robotics.swerveio.SwerveMeta;
import net.bancino.robotics.swerveio.SwerveModule;
import net.bancino.robotics.swerveio.encoder.AbstractEncoder;
import net.bancino.robotics.swerveio.encoder.AnalogEncoder;
import net.bancino.robotics.swerveio.encoder.SparkMaxEncoder;
import net.bancino.robotics.swerveio.exception.SwerveException;
import net.bancino.robotics.swerveio.exception.SwerveRuntimeException;
import net.bancino.robotics.swerveio.geometry.ChassisDimension;
import net.bancino.robotics.swerveio.geometry.Length;
import net.bancino.robotics.swerveio.geometry.Unit;
import net.bancino.robotics.swerveio.gyro.AbstractGyro;
import net.bancino.robotics.swerveio.kinematics.DefaultSwerveKinematics;
import net.bancino.robotics.swerveio.kinematics.SwerveKinematicsProvider;
import net.bancino.robotics.swerveio.log.DashboardSwerveLogger;
import net.bancino.robotics.swerveio.module.AbstractSwerveModule;
import frc.robot.modules.MK2Swerve;
import net.bancino.robotics.swerveio.pid.AbstractPIDController;
import net.bancino.robotics.swerveio.gyro.NavXGyro;

/**
 * The drivetrain subsystem drives the robot! (wow!).
 *
 * This subsystem consists of the following components:
 * - Swerve module (4x drive + pivot motor)
 *
 * This subsystem should provide the following functions:
 * - Run the drivetrain with joystick
 * - Run the drivetrain autonomously
 */
public class DriveTrain extends SwerveDrive {

	// private static final double STEER_P = 0.008, STEER_I = 0.00000155, STEER_D = 0.0;
  
	private static final double STEER_P = 0.008, STEER_I = 0.0, STEER_D = 0.00008;
  private static final double RAMP_RATE = 0.5;
  
  private static DriveTrain instance;

  private DriveTrain(AbstractGyro gyro) throws SwerveException {
    super(new SwerveMeta() {

      private final AbstractEncoder frontRightEncoder = new AnalogEncoder(Constants.DRIVE_FRONT_RIGHT_STEER_ENCODER, Constants.DRIVE_FRONT_RIGHT_STEER_OFFSET);
      private final AbstractEncoder frontLeftEncoder = new AnalogEncoder(Constants.DRIVE_FRONT_LEFT_STEER_ENCODER, Constants.DRIVE_FRONT_LEFT_STEER_OFFSET);
      private final AbstractEncoder rearLeftEncoder = new AnalogEncoder(Constants.DRIVE_BACK_LEFT_STEER_ENCODER, Constants.DRIVE_BACK_LEFT_STEER_OFFSET);
      private final AbstractEncoder rearRightEncoder = new AnalogEncoder(Constants.DRIVE_BACK_RIGHT_STEER_ENCODER, Constants.DRIVE_BACK_RIGHT_STEER_OFFSET);

      @Override
      public String name() {
        return "Honey Badger";
      }

      @Override
      public SwerveKinematicsProvider kinematicsProvider() {
        return new DefaultSwerveKinematics(new ChassisDimension(new Length(20, Unit.INCHES), new Length(24, Unit.INCHES)));
      }

      @Override
      public double countsPerPivotRevolution() {
        return 360;
      }

      @Override
      public Map<SwerveModule, AbstractSwerveModule> moduleMap() {
        var modules = new HashMap<SwerveModule, AbstractSwerveModule>();
        modules.put(SwerveModule.FRONT_RIGHT, 
          new MK2Swerve(Constants.DRIVE_FRONT_RIGHT_DRIVE_MOTOR, Constants.DRIVE_FRONT_RIGHT_STEER_MOTOR, frontRightEncoder));
        modules.put(SwerveModule.FRONT_LEFT,
          new MK2Swerve(Constants.DRIVE_FRONT_LEFT_DRIVE_MOTOR, Constants.DRIVE_FRONT_LEFT_STEER_MOTOR, frontLeftEncoder));
        modules.put(SwerveModule.REAR_LEFT,
          new MK2Swerve(Constants.DRIVE_BACK_LEFT_DRIVE_MOTOR, Constants.DRIVE_BACK_LEFT_STEER_MOTOR, rearLeftEncoder));
        modules.put(SwerveModule.REAR_RIGHT,
          new MK2Swerve(Constants.DRIVE_BACK_RIGHT_DRIVE_MOTOR, Constants.DRIVE_BACK_RIGHT_STEER_MOTOR, rearRightEncoder));
        return modules; /* Return the module map for the constructor's use. */
      }

      @Override
      public AbstractGyro gyro() {
        gyro.zero();
        return gyro;
      }

      @Override
      public void modifyModule(AbstractSwerveModule module) {
        AbstractPIDController pivotModulePid = module.getPivotPIDController();
        pivotModulePid.setOutputRampRate(RAMP_RATE);
        pivotModulePid.setPID(STEER_P, STEER_I, STEER_D);
      }

      @Override
      public List<SwerveFlag> applyFlags() {
        return List.of(
          SwerveFlag.ENABLE_PIVOT_OPTIMIZE
        );
      }

      @Override
      public void initialize(SwerveDrive swerve) {
        swerve.zeroDriveEncoders();
        swerve.setFieldCentric(true);
        
        // swerve.setIdleAngle(0, false);

        // TODO: Only on when debugging drive/pivot motors
        // swerve.startLogging(new DashboardSwerveLogger());

        // File logFile = new File("pid.csv");
        // try {
        //   logFile.createNewFile();
        //   swerve.startLogging(100, new CSVPIDSwerveLogger(logFile, SwerveModule.FRONT_LEFT));
        // } catch (IOException e) {
        //   System.out.println("Error Creating Robot CSV: " + e);
        //   e.printStackTrace();
        // }
      }

    });
  }

  public static DriveTrain getInstance(AbstractGyro gyro) {
    if (instance == null) {
      try {
        instance = new DriveTrain(gyro);
      } catch (SwerveException e) {
        throw new SwerveRuntimeException(e);
      }
    }
    return instance;
  }
}