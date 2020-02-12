/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;
import frc.robot.Constants;
import frc.robot.Robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANError;




public class Indexer extends SubsystemBase {

  public static CANSparkMax IndexerMotorOne;
  public static double IndexerSpeedMotorOne = 0.25;
  public final double INDEXER_SPEED = 0.25;
  public final int INTERVAL_OF_STOPS = 100;
  public static double UntilStop;
  
  private CommandScheduler scheduler;
  /**
   * Creates a new ExampleSubsystem.
   */
  public Indexer () {

    scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(this);

    IndexerMotorOne = new CANSparkMax(Constants.INDEXER_VICTOR_ONE, MotorType.kBrushless);
    IndexerMotorOne.restoreFactoryDefaults();
    IndexerSpeedMotorOne = 0;
  }

  public void CounterShutDown(){
    double PreviousSpeed = IndexerSpeedMotorTwo;
    UntilStop += 1;
    if (UntilStop > INTERVAL_OF_STOPS)
    {
        IndexerSpeedMotorTwo = 0;
        LengthOfPause();
        IndexerSpeedMotorTwo = PreviousSpeed;
    }
  }

  public void LengthOfPause(){
    UntilStop -= 2;
    if (UntilStop > 0)
    {
        IndexerSpeedMotorTwo = 0;
        RunMotorTwo();
        LengthOfPause();
    }
  }

  public static void RunMotors(){
    IndexerMotorOne.set(IndexerSpeedMotorOne);
    RunMotorTwo();
  } 

  public static void RunMotorTwo(){
    IndexerMotorTwo.set(IndexerSpeedMotorTwo);
  }   

  public static void StopMotors(){
    IndexerMotorOne.set(0);
    IndexerMotorTwo.set(0);
  }    

  public void StopMotorTwo(){
    IndexerMotorTwo.set(0);
  }   
}
