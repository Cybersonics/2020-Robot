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

import com.ctre.phoenix.motorcontrol.*;

public class Shooter extends SubsystemBase {

  //PWMSparkMax intakeMotor;
  public double ShooterRate;
  public double PivotRate = 0.5;
  public static CANSparkMax ShooterMotorOne;
  public static CANSparkMax ShooterMotorTwo;
  public static TalonSRX PivotMotor;

  private CommandScheduler scheduler;

  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
    ShooterRate = 0;
    //ShooterMotorOne = new ShooterMotorOne();
    scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(this);
    ShooterMotorOne = new CANSparkMax(Constants.SHOOTER_SPARK_ONE, MotorType.kBrushless);//maybe parameter needs to be changed
    ShooterMotorOne.restoreFactoryDefaults();
    ShooterMotorTwo = new CANSparkMax(Constants.SHOOTER_SPARK_TWO, MotorType.kBrushless);//maybe parameter needs to be changed
    ShooterMotorTwo.restoreFactoryDefaults();
    PivotMotor = new TalonSRX(Constants.PIVOT_SPARK);
    PivotMotor.configFactoryDefault();
  }

  public void ShooterRun() {
    ShooterMotorOne.set(-ShooterRate);
    ShooterMotorTwo.set(ShooterRate);
    //Shutdown seqeunce
    if(ShooterRate>0.0)
    {
        // Might be used if necessary (Timer Thing)
        // indexer.CounterShutDown();
        Indexer.RunMotors();
    }
    else if (Intake.GetIntakeInValue() == 0) {
      Indexer.StopMotors();
    }

  }

  public void SetShooterRate(double EnteredRate) {
    ShooterRate = EnteredRate;
  }

  public void PivotUp() {
    PivotMotor.set(ControlMode.PercentOutput, PivotRate);
  }

  public void PivotDown() {
    PivotMotor.set(ControlMode.PercentOutput, -PivotRate);
  }

  public void PivotShutDown() {
    PivotMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

