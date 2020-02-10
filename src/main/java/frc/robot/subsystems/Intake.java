/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import org.usfirst.frc103.Robot2020.Robot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.*;

public class Intake extends SubsystemBase {
  private static TalonSRX intakeMotor;
  public static double IntakeIn;
  public double IntakeOut;
  public final double DEADZONE = 0.07;
  public final double RAMP_UP_RATE = 0.3;
  public final double RAMP_DOWN_RATE = 0.25;
  public final int INTERVAL_OF_STOPS = 100;
  public double UntilStop;

  private CommandScheduler scheduler;


  /**
   * Creates a new ExampleSubsystem.
   */
  public Intake () {

    scheduler = CommandScheduler.getInstance();
    scheduler.registerSubsystem(this);
    
    intakeMotor = new TalonSRX(Constants.INTAKE_TALON);
    intakeMotor.configFactoryDefault();
    IntakeIn = 0;
    IntakeOut = 0;
  }

  public void intakeRun() {
    if ((IntakeIn > 0 && IntakeOut > 0) || (IntakeIn < DEADZONE && IntakeOut < DEADZONE)) 
    {
      intakeMotor.set(ControlMode.PercentOutput, 0.0);
    } 
    else 
    {
      if ((IntakeIn > 0)&&(Robot.xBoxController.getYButton()))
      {
         intakeMotor.set(ControlMode.PercentOutput, -IntakeIn);
      } 
      else if (IntakeIn > 0)
      {
          intakeMotor.set(ControlMode.PercentOutput, -(IntakeIn * RAMP_UP_RATE));
      }
    }
    if (IntakeOut > 0)
    {
        intakeMotor.set(ControlMode.PercentOutput, IntakeOut * RAMP_DOWN_RATE);
    }
  }

  
  public void IntakeInIncrease()
  {
    IntakeIn+=0.1;
  }

  public void IntakeInDecrease()
  {
    IntakeIn-=0.1;
  }

  public void IntakeOutIncrease()
  {
    IntakeOut+=0.1;
  }

  public void IntakeOutDecrease()
  {
    IntakeIn-=0.1;
  }

  public void SetIntakes(double IntakeIn, double IntakeOut){
    this.IntakeIn = IntakeIn;
    this.IntakeOut = IntakeOut;
  }

  // Moving to Sewcond Motor on Indexer
  /*
  public void CounterShutDown(){
    double PreviousIntakeIn = IntakeIn;
    UntilStop += 1;
    if (UntilStop > INTERVAL_OF_STOPS)
    {
      IntakeIn = 0;
      Robot.intake.LengthOfPause();
      IntakeIn = PreviousIntakeIn;
    }
  }

  public void LengthOfPause(){
    UntilStop -= 2;
    if (UntilStop > 0)
    {
      IntakeIn = 0;
      Robot.intake.LengthOfPause();
    }
  }
  */

  public static double GetIntakeInValue(){
  return IntakeIn;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

