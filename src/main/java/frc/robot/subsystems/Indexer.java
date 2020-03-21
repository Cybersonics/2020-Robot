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
import com.revrobotics.CANError;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;




public class Indexer extends SubsystemBase {

  public static VictorSPX IndexerMotor;
  public final int INTERVAL_OF_STOPS = 100;
  public static double UntilStop;
  
  private CommandScheduler scheduler;
  /**
   * Creates a new ExampleSubsystem.
   */
  public Indexer () {

    scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(this);

    IndexerMotor = new VictorSPX(Constants.INDEXER_VICTOR);
    IndexerMotor.configFactoryDefault();

  }

  public void runElevator(double speed){
    IndexerMotor.set(ControlMode.PercentOutput, speed);
  } 

  public void stopIndexer(){
    IndexerMotor.set(ControlMode.PercentOutput,0);
  }    
 
}
