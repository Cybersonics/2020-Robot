package frc.robot.subsystems;

import static frc.robot.Constants.DRIVE_BACK_LEFT_DRIVE_MOTOR;
import static frc.robot.Constants.DRIVE_BACK_LEFT_STEER_ENCODER;
import static frc.robot.Constants.DRIVE_BACK_LEFT_STEER_MOTOR;
import static frc.robot.Constants.DRIVE_BACK_LEFT_STEER_OFFSET;
import static frc.robot.Constants.DRIVE_BACK_RIGHT_DRIVE_MOTOR;
import static frc.robot.Constants.DRIVE_BACK_RIGHT_STEER_ENCODER;
import static frc.robot.Constants.DRIVE_BACK_RIGHT_STEER_MOTOR;
import static frc.robot.Constants.DRIVE_BACK_RIGHT_STEER_OFFSET;
import static frc.robot.Constants.DRIVE_FRONT_LEFT_DRIVE_MOTOR;
import static frc.robot.Constants.DRIVE_FRONT_LEFT_STEER_ENCODER;
import static frc.robot.Constants.DRIVE_FRONT_LEFT_STEER_MOTOR;
import static frc.robot.Constants.DRIVE_FRONT_LEFT_STEER_OFFSET;
import static frc.robot.Constants.DRIVE_FRONT_RIGHT_DRIVE_MOTOR;
import static frc.robot.Constants.DRIVE_FRONT_RIGHT_STEER_ENCODER;
import static frc.robot.Constants.DRIVE_FRONT_RIGHT_STEER_MOTOR;
import static frc.robot.Constants.DRIVE_FRONT_RIGHT_STEER_OFFSET;

import java.util.Optional;

import com.google.errorprone.annotations.concurrent.GuardedBy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.frcteam2910.common.control.*;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.kinematics.ChassisVelocity;
import org.frcteam2910.common.kinematics.SwerveKinematics;
import org.frcteam2910.common.kinematics.SwerveOdometry;
import org.frcteam2910.common.math.RigidTransform2;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.UpdateManager;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.util.DrivetrainFeedforwardConstants;
import org.frcteam2910.common.util.HolonomicDriveSignal;
import org.frcteam2910.common.util.HolonomicFeedforward;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase implements UpdateManager.Updatable {

        private static final double MAX_VELOCITY = 12.0 * 12.0;

        public static final TrajectoryConstraint[] CONSTRAINTS = { 
                new MaxVelocityConstraint(MAX_VELOCITY),
                new MaxAccelerationConstraint(13.0 * 12.0),
                new CentripetalAccelerationConstraint(25.0 * 12.0) };

        private static final double TRACKWIDTH = 23.5;
        private static final double WHEELBASE = 22.0;
        private static final double DRIVE_REDUCTION = 9.62 / 1.0;
        private static final double WHEEL_DIAMETER = 4.0;

        private static CANSparkMax LF_Drive = new CANSparkMax(DRIVE_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);

        private static CANSparkMax RR_Drive = new CANSparkMax(DRIVE_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
        private static CANSparkMax RF_Drive = new CANSparkMax(DRIVE_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);

        private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
                        new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                                        .angleEncoder(new AnalogInput(DRIVE_FRONT_LEFT_STEER_ENCODER),
                                                        DRIVE_FRONT_LEFT_STEER_OFFSET)
                                        .angleMotor(new CANSparkMax(DRIVE_FRONT_LEFT_STEER_MOTOR, MotorType.kBrushless),
                                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                                        .driveMotor(LF_Drive, DRIVE_REDUCTION, WHEEL_DIAMETER).build();
        private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
                        new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                                        .angleEncoder(new AnalogInput(DRIVE_FRONT_RIGHT_STEER_ENCODER),
                                                        DRIVE_FRONT_RIGHT_STEER_OFFSET)
                                        .angleMotor(new CANSparkMax(DRIVE_FRONT_RIGHT_STEER_MOTOR,
                                                        MotorType.kBrushless), Mk2SwerveModuleBuilder.MotorType.NEO)
                                        .driveMotor(RF_Drive, DRIVE_REDUCTION, WHEEL_DIAMETER).build();
        private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
                        new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
                                        .angleEncoder(new AnalogInput(DRIVE_BACK_LEFT_STEER_ENCODER),
                                                        DRIVE_BACK_LEFT_STEER_OFFSET)
                                        .angleMotor(new CANSparkMax(DRIVE_BACK_LEFT_STEER_MOTOR, MotorType.kBrushless),
                                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                                        .driveMotor(new CANSparkMax(DRIVE_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless),
                                                        DRIVE_REDUCTION, WHEEL_DIAMETER)
                                        .build();
        private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
                        new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
                                        .angleEncoder(new AnalogInput(DRIVE_BACK_RIGHT_STEER_ENCODER),
                                                        DRIVE_BACK_RIGHT_STEER_OFFSET)
                                        .angleMotor(new CANSparkMax(DRIVE_BACK_RIGHT_STEER_MOTOR, MotorType.kBrushless),
                                                        Mk2SwerveModuleBuilder.MotorType.NEO)
                                        .driveMotor(RR_Drive, DRIVE_REDUCTION, WHEEL_DIAMETER).build();
        private final SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

        private final SwerveKinematics kinematics = new SwerveKinematics(new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Front
                                                                                                                         // Left
                        new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0), // Front Right
                        new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0), // Back Left
                        new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0) // Back Right
        );
        private final SwerveOdometry odometry = new SwerveOdometry(kinematics, RigidTransform2.ZERO);

        private final Object sensorLock = new Object();
        @GuardedBy("sensorLock")
        private final NavX navX = new NavX(SPI.Port.kMXP);

        private final Object kinematicsLock = new Object();
        @GuardedBy("kinematicsLock")
        private RigidTransform2 pose = RigidTransform2.ZERO;

        private final Object stateLock = new Object();
        @GuardedBy("stateLock")
        private HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(Vector2.ZERO, 0.0, false);;

        // Logging stuff
        private NetworkTableEntry poseXEntry;
        private NetworkTableEntry poseYEntry;
        private NetworkTableEntry poseAngleEntry;

        private NetworkTableEntry[] moduleAngleEntries = new NetworkTableEntry[modules.length];

        private static DrivetrainSubsystem instance;

        private PidController snapRotationController = new PidController(SNAP_ROTATION_CONSTANTS);
        private double snapRotation = Double.NaN;

        private DrivetrainSubsystem() {
                synchronized (sensorLock) {
                        this.navX.setInverted(true);
                }
                LF_Drive.setInverted(true);
                RR_Drive.setInverted(true);
                RF_Drive.setInverted(true);

                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                poseXEntry = tab.add("Pose X", 0.0).withPosition(0, 0).withSize(1, 1).getEntry();
                poseYEntry = tab.add("Pose Y", 0.0).withPosition(0, 1).withSize(1, 1).getEntry();
                poseAngleEntry = tab.add("Pose Angle", 0.0).withPosition(0, 2).withSize(1, 1).getEntry();

                ShuffleboardLayout frontLeftModuleContainer = tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                                .withPosition(1, 0).withSize(2, 3);
                moduleAngleEntries[0] = frontLeftModuleContainer.add("Angle", 0.0).getEntry();

                ShuffleboardLayout frontRightModuleContainer = tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                                .withPosition(3, 0).withSize(2, 3);
                moduleAngleEntries[1] = frontRightModuleContainer.add("Angle", 0.0).getEntry();

                ShuffleboardLayout backLeftModuleContainer = tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                                .withPosition(5, 0).withSize(2, 3);
                moduleAngleEntries[2] = backLeftModuleContainer.add("Angle", 0.0).getEntry();

                ShuffleboardLayout backRightModuleContainer = tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                                .withPosition(7, 0).withSize(2, 3);
                moduleAngleEntries[3] = backRightModuleContainer.add("Angle", 0.0).getEntry();

                snapRotationController.setInputRange(0.0, 2.0 * Math.PI);
                snapRotationController.setContinuous(true);
                snapRotationController.setOutputRange(-0.5, 0.5);
        }

        public static DrivetrainSubsystem getInstance() {
                if (instance == null) {
                        instance = new DrivetrainSubsystem();
                }
                return instance;
        }

        public RigidTransform2 getPose() {
                synchronized (kinematicsLock) {
                        return pose;
                }
        }

        public void drive(Vector2 translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
                synchronized (stateLock) {
                        driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, true);
                }
        }

        public void resetGyroAngle(Rotation2 angle) {
                synchronized (sensorLock) {
                        this.navX.setAdjustmentAngle(this.navX.getUnadjustedAngle().rotateBy(angle.inverse()));
                }
        }

        public SwerveModule[] getSwerveModules() {
                return modules;
        }

        @Override
        public void update(double timestamp, double dt) {
                updateOdometry(dt);
                
                HolonomicDriveSignal driveSignal;

                synchronized (stateLock) {
                        synchronized (sensorLock) {
                                Optional<HolonomicDriveSignal> optSignal = follower.update(getPose(),
                                                this.driveSignal.getTranslation(), this.navX.getRate(), timestamp, dt);

                                if (optSignal.isPresent()) {
                                        driveSignal = optSignal.get();
                                } else {
                                        synchronized (stateLock) {
                                                driveSignal = this.driveSignal;
                                        }      
                                }
                        }
                }
                
                updateModules(driveSignal, dt);

        }

        private void updateOdometry(double dt) {
                Vector2[] moduleVelocities = new Vector2[modules.length];
                for (int i = 0; i < modules.length; i++) {
                        var module = modules[i];
                        module.updateSensors();

                        moduleVelocities[i] = Vector2.fromAngle(Rotation2.fromRadians(module.getCurrentAngle()))
                                        .scale(module.getCurrentVelocity());
                }

                Rotation2 angle;
                synchronized (sensorLock) {
                        angle = navX.getAngle();
                }

                RigidTransform2 pose = odometry.update(angle, dt, moduleVelocities);

                synchronized (kinematicsLock) {
                        this.pose = pose;
                }
        }

        private void updateModules(HolonomicDriveSignal signal, double dt) {
                ChassisVelocity velocity;
                if (signal == null) {
                        velocity = new ChassisVelocity(Vector2.ZERO, 0.0);
                } else if (signal.isFieldOriented()) {
                        velocity = new ChassisVelocity(signal.getTranslation().rotateBy(getPose().rotation.inverse()),
                                        signal.getRotation());
                } else {
                        velocity = new ChassisVelocity(signal.getTranslation(), signal.getRotation());
                }

                Vector2[] moduleOutputs = kinematics.toModuleVelocities(velocity);
                SwerveKinematics.normalizeModuleVelocities(moduleOutputs, 1.0);

                for (int i = 0; i < modules.length; i++) {
                        var module = modules[i];
                        module.setTargetVelocity(moduleOutputs[i]);
                        module.updateState(dt);
                }
        }

        @Override
        public void periodic() {
                var pose = getPose();
                poseXEntry.setDouble(pose.translation.x);
                poseYEntry.setDouble(pose.translation.y);
                poseAngleEntry.setDouble(pose.rotation.toDegrees());

                for (int i = 0; i < modules.length; i++) {
                        var module = modules[i];
                        moduleAngleEntries[i].setDouble(Math.toDegrees(module.getCurrentAngle()));
                }
        }

        private static final PidConstants FOLLOWER_TRANSLATION_CONSTANTS = new PidConstants(0.05, 0.01, 0.0);
        private static final PidConstants FOLLOWER_ROTATION_CONSTANTS = new PidConstants(0.2, 0.01, 0.0);
        private static final HolonomicFeedforward FOLLOWER_FEEDFORWARD_CONSTANTS = new HolonomicFeedforward(
                        new DrivetrainFeedforwardConstants(1.0 / (14.0 * 12.0), 0.0, 0.0));

        private static final PidConstants SNAP_ROTATION_CONSTANTS = new PidConstants(0.3, 0.01, 0.0);

        private HolonomicMotionProfiledTrajectoryFollower follower = new HolonomicMotionProfiledTrajectoryFollower(
                        FOLLOWER_TRANSLATION_CONSTANTS, FOLLOWER_ROTATION_CONSTANTS, FOLLOWER_FEEDFORWARD_CONSTANTS);

        public HolonomicMotionProfiledTrajectoryFollower getFollower() {
                return follower;
        }

        public void setSnapRotation(double snapRotation) {
                synchronized (stateLock) {
                        this.snapRotation = snapRotation;
                }
        }
}