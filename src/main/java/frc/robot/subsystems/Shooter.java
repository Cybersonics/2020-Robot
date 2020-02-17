/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  public double ShooterRate;
  public double PivotRate = 0.5;
  public static CANSparkMax ShooterMotorOne;
  public static CANSparkMax ShooterMotorTwo;
  public static TalonSRX PivotMotor;
  public static AnalogEncoder Encoder;
  private static AnalogInput EncoderInput;
  private static boolean EncoderStart;

  public double CurrentAngle;
  public double IncreaseOfAngle;

  private CommandScheduler scheduler;


  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
    ShooterRate = 1.0;
    EncoderStart = true;
    scheduler = CommandScheduler.getInstance();
    ShooterMotorOne = new CANSparkMax(Constants.SHOOTER_SPARK_ONE, MotorType.kBrushless);//maybe parameter needs to be changed
    ShooterMotorOne.restoreFactoryDefaults();
    ShooterMotorTwo = new CANSparkMax(Constants.SHOOTER_SPARK_TWO, MotorType.kBrushless);//maybe parameter needs to be changed
    ShooterMotorTwo.restoreFactoryDefaults();
    PivotMotor = new TalonSRX(Constants.PIVOT_TALON);
    PivotMotor.configFactoryDefault();
    // EncoderInput = new AnalogInput(Constants.SHOOTER_ENCODER);//needs to be changed
    // Encoder = new AnalogEncoder(EncoderInput);
    // Encoder.setDistancePerRotation(Constants.SHOOTER_ENCODER_ANGLE_INCREASE);//set the distance to the increase in angle per rotation
    // Encoder.reset();
    scheduler.registerSubsystem(this);
  }

  public void launch() {
    ShooterMotorOne.set(ShooterRate);
    ShooterMotorTwo.set(-ShooterRate);
  }

  public void curve() {
    ShooterMotorOne.set(ShooterRate*.2);
    ShooterMotorTwo.set(-ShooterRate);
  }

  public void stop() {
    ShooterMotorOne.set(0);
    ShooterMotorTwo.set(0);
  }

  public void SetShooterRate(double EnteredRate) {
    ShooterRate = EnteredRate;
  }

  public void PivotUp() {
    // if (Encoder.getDistance() < Constants.MAX_ANGLE)// needs to be updated
    //   PivotMotor.set(ControlMode.PercentOutput, PivotRate);
    // CurrentAngle += IncreaseOfAngle;
  }

  public void PivotDown() {
    // if (Encoder.getDistance() > Constants.MIN_ANGLE)// needs to be updated
    //   PivotMotor.set(ControlMode.PercentOutput, -PivotRate);
    // CurrentAngle -= IncreaseOfAngle;
  }

  public void PivotShutDown() {
    // PivotMotor.set(ControlMode.PercentOutput, 0);
  }

  public void SetAngle(double NewAngle){
//     //moves motor down till hits angle
//     while(Encoder.getDistance()>NewAngle){ //might need to add an acceptable range condition so the the motor doesn't fluctuate if the value is not exact
//       PivotDown();
//     }
//     //stops motor
//     PivotShutDown();
//     //moves motor up till hit angle
//     while(Encoder.getDistance()<NewAngle){ //might need to add an acceptable range condition so the the motor doesn't fluctuate if the value is not exact
//       PivotUp();
//     }
//     CurrentAngle = NewAngle;
//   }

//   public void ZeroAngle(){
//     if (EncoderStart){
//       PivotDown();
//       Encoder.reset();
//       SetAngle(3.0);
//       Encoder.reset();
//     }
//     else{
//       SetAngle(0.0);
//       Encoder.reset();
//     }
//     CurrentAngle = 0.0;
//     EncoderStart = false;
  }

  //Set the angle as minimum
  public void AngleIsMinimum() {

    // CurrentAngle = Constants.MIN_ANGLE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

