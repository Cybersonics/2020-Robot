/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj.command.Command;
//import org.usfirst.frc103.Robot2020.Robot;
import frc.robot.Constants; 
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANError;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import com.ctre.phoenix.motorcontrol.*;

public class Shooter extends SubsystemBase {

  //PWMSparkMax intakeMotor;
  public double ShooterRate;
  public double PivotRate = 0.5;
  public CANSparkMax ShooterMotorOne;
  public CANSparkMax ShooterMotorTwo;
  public TalonSRX PivotMotor;
  public AnalogEncoder Encoder;
  private AnalogInput EncoderInput;
  private boolean EncoderStart;

  public double CurrentAngle;
  public double IncreaseOfAngle;

  private CommandScheduler scheduler;

  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
  //   ShooterRate = 0;
  //   EncoderStart = true;
  //   //ShooterMotorOne = new ShooterMotorOne();
  //   scheduler = CommandScheduler.getInstance();
  //   scheduler.registerSubsystem(this);
  //   ShooterMotorOne = new CANSparkMax(Constants.SHOOTER_SPARK_ONE, MotorType.kBrushless);//maybe parameter needs to be changed
  //   ShooterMotorOne.restoreFactoryDefaults();
  //   ShooterMotorTwo = new CANSparkMax(Constants.SHOOTER_SPARK_TWO, MotorType.kBrushless);//maybe parameter needs to be changed
  //   ShooterMotorTwo.restoreFactoryDefaults();
  //   PivotMotor = new TalonSRX(Constants.PIVOT_TALON);
  //   PivotMotor.configFactoryDefault();
  //   EncoderInput = new AnalogInput(Constants.SHOOTER_ENCODER);//needs to be changed
  //   Encoder = new AnalogEncoder(EncoderInput);
  //   Encoder.setDistancePerRotation(Constants.SHOOTER_ENCODER_ANGLE_INCREASE);//set the distance to the increase in angle per rotation
  //   Encoder.reset();
  // }

  // public void ShooterRun() {
  //   ShooterMotorOne.set(-ShooterRate);
  //   ShooterMotorTwo.set(ShooterRate);
  //   //Shutdown seqeunce
  //   if(ShooterRate>0.0)
  //   {
  //       // Might be used if necessary (Timer Thing)
  //       // indexer.CounterShutDown();
  //       Indexer.RunMotor();
  //   }


  // }

  // public void SetShooterRate(double EnteredRate) {
  //   ShooterRate = EnteredRate;
  // }

  // public void PivotUp() {
  //   if (Encoder.getDistance() < Constants.MAX_ANGLE)// needs to be updated
  //     PivotMotor.set(ControlMode.PercentOutput, PivotRate);
  //   CurrentAngle += IncreaseOfAngle;
  // }

  // public void PivotDown() {
  //   if (Encoder.getDistance() > Constants.MIN_ANGLE)// needs to be updated
  //     PivotMotor.set(ControlMode.PercentOutput, -PivotRate);
  //   CurrentAngle -= IncreaseOfAngle;
  // }

  // public void PivotShutDown() {
  //   PivotMotor.set(ControlMode.PercentOutput, 0);
  // }

  // public void SetAngle(double NewAngle){
  //   //moves motor down till hits angle
  //   while(Encoder.getDistance()>NewAngle){ //might need to add an acceptable range condition so the the motor doesn't fluctuate if the value is not exact
  //     PivotDown();
  //   }
  //   //stops motor
  //   PivotShutDown();
  //   //moves motor up till hit angle
  //   while(Encoder.getDistance()<NewAngle){ //might need to add an acceptable range condition so the the motor doesn't fluctuate if the value is not exact
  //     PivotUp();
  //   }
  //   CurrentAngle = NewAngle;
  // }

  // public void ZeroAngle(){
  //   if (EncoderStart){
  //     PivotDown();
  //     Encoder.reset();
  //     SetAngle(3.0);
  //     Encoder.reset();
  //   }
  //   else{
  //     SetAngle(0.0);
  //     Encoder.reset();
  //   }
  //   CurrentAngle = 0.0;
  //   EncoderStart = false;
  // }

  // //Set the angle as minimum
  // public void AngleIsMinimum() {

  //   CurrentAngle = Constants.MIN_ANGLE;
  // }

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
   }
}

