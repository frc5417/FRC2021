/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.subsystems.*;

/**
 * Add your docs here.
 */
public class Drive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public CANSparkMax driveSlaveL = new CANSparkMax(Constants.slaveLeftMotor, MotorType.kBrushless);
  public CANSparkMax driveMasterR = new CANSparkMax(Constants.masterRightMotor, MotorType.kBrushless);
  public CANSparkMax driveSlaveR = new CANSparkMax(Constants.slaveRightMotor, MotorType.kBrushless);
  public CANSparkMax driveMasterL = new CANSparkMax(Constants.masterLeftMotor, MotorType.kBrushless);
  TalonSRX turretMotor = new TalonSRX(9);
  public Solenoid shifter;
  Compressor compressor;
  Limelight limelight = new Limelight();
  
  boolean shifterFlag = false;
  boolean toggle = true;

  /*TalonSRX driveMasterRight = new TalonSRX(Constants.masterRightMotor);
  TalonSRX driveMasterLeft = new TalonSRX(Constants.masterLeftMotor);
  VictorSPX driveSlaveRight = new VictorSPX(Constants.slaveRightMotor);
  VictorSPX driveSlaveLeft = new VictorSPX(Constants.slaveLeftMotor);*/


  public Drive(){
  
/*
    driveMasterRight.setNeutralMode(NeutralMode.Coast);
    driveMasterLeft.setNeutralMode(NeutralMode.Coast);
    driveSlaveRight.setNeutralMode(NeutralMode.Coast);
    driveSlaveLeft.setNeutralMode(NeutralMode.Coast);
*/

    turretMotor.setNeutralMode(NeutralMode.Coast);



    driveMasterL.getPIDController().setP(Constants.drivekP);
    driveMasterR.getPIDController().setP(Constants.drivekP);
    driveSlaveL.getPIDController().setP(Constants.drivekP);
    driveSlaveR.getPIDController().setP(Constants.drivekP);
    driveMasterL.getPIDController().setI(Constants.drivekI);
    driveMasterR.getPIDController().setI(Constants.drivekI);
    driveSlaveL.getPIDController().setI(Constants.drivekI);
    driveSlaveR.getPIDController().setI(Constants.drivekI);

    driveMasterL.getEncoder().setPosition(0);
    driveMasterR.getEncoder().setPosition(0);
    driveSlaveL.getEncoder().setPosition(0);
    driveSlaveR.getEncoder().setPosition(0);
    
    shifter = new Solenoid(0);

    
  }
  

  public void SetPower(double leftPower, double rightPower){
    driveMasterL.setIdleMode(IdleMode.kCoast);
    driveMasterR.setIdleMode(IdleMode.kCoast);
    driveSlaveL.setIdleMode(IdleMode.kCoast);
    driveSlaveR.setIdleMode(IdleMode.kCoast);
    //if (!((leftPower < .1)&&(leftPower > -.1) || (rightPower < .1)&&(rightPower > -.1))){
    /*  
    driveMasterLeft.set(ControlMode.PercentOutput, -leftPower);
    driveMasterRight.set(ControlMode.PercentOutput, -rightPower);
    driveSlaveLeft.set(ControlMode.PercentOutput, leftPower);
    driveSlaveRight.set(ControlMode.PercentOutput, -rightPower);
    */

    driveMasterL.set(-Math.pow(leftPower, 3));
    driveMasterR.set(Math.pow(rightPower, 3)); //inversions for bowser
    driveSlaveL.set(-Math.pow(leftPower, 3));
    driveSlaveR.set(Math.pow(rightPower, 3));
    //System.out.println("Left Speed: " + driveMasterL.getEncoder().getVelocity());
    //System.out.println("Right Speed: " + driveMasterR.getEncoder().getVelocity());
    
  }
    /*else{
      driveMasterLeft.set(ControlMode.PercentOutput, 0);
      driveMasterRight.set(ControlMode.PercentOutput, 0);
      driveSlaveLeft.set(ControlMode.PercentOutput, 0);
      driveSlaveRight.set(ControlMode.PercentOutput, 0);
    }*/
  
    
    public void autoPower(double leftPower, double rightPower){
    
      /*driveMasterLeft.set(ControlMode.PercentOutput, -leftPower);
      driveMasterRight.set(ControlMode.PercentOutput, -rightPower);
      driveSlaveLeft.set(ControlMode.PercentOutput, leftPower);
      driveSlaveRight.set(ControlMode.PercentOutput, -rightPower);*/
      
      driveMasterL.set(leftPower);
      driveMasterR.set(rightPower);
      driveSlaveL.set(leftPower);
      driveSlaveR.set(rightPower);
      
  
    }

      
  
  public void autoPowerAuton(double driveMasterLPos, double driveMasterRPos, double driveSlaveLPos, double driveSlaveRPos){
    
    /*driveMasterLeft.set(ControlMode.PercentOutput, -leftPower);
    driveMasterRight.set(ControlMode.PercentOutput, -rightPower);
    driveSlaveLeft.set(ControlMode.PercentOutput, leftPower);
    driveSlaveRight.set(ControlMode.PercentOutput, -rightPower);*/
    /*
    driveMasterL.set(leftPower);
    driveMasterR.set(rightPower);
    driveSlaveL.set(leftPower);
    driveSlaveR.set(rightPower);
    */
    driveMasterL.setIdleMode(IdleMode.kBrake);
    driveMasterR.setIdleMode(IdleMode.kBrake);
    driveSlaveL.setIdleMode(IdleMode.kBrake);
    driveSlaveR.setIdleMode(IdleMode.kBrake);

    driveMasterL.getPIDController().setReference(driveMasterLPos, ControlType.kPosition);
    driveMasterR.getPIDController().setReference(driveMasterRPos, ControlType.kPosition);
    driveSlaveL.getPIDController().setReference(driveSlaveLPos, ControlType.kPosition);
    driveSlaveR.getPIDController().setReference(driveSlaveRPos, ControlType.kPosition);
/*
    System.out.println("Master Motor Right Pos: " + driveMasterR.getEncoder().getPosition());
    System.out.println("Master Motor Left Pos: " + driveMasterL.getEncoder().getPosition());
    System.out.println("Slave Motor Right Pos: " + driveSlaveR.getEncoder().getPosition());
    System.out.println("Slave Motor Left Pos: " + driveMasterL.getEncoder().getPosition());
*/

  }

  public void setDriveTrainRampRate(double rightRampRate, double leftRampRate){
    driveMasterR.setClosedLoopRampRate(rightRampRate);
    driveMasterL.setClosedLoopRampRate(leftRampRate);
    driveSlaveR.setClosedLoopRampRate(rightRampRate);
    driveSlaveL.setClosedLoopRampRate(leftRampRate);
  }

  public void zeroReset(){
    driveMasterL.getEncoder().setPosition(0);
    driveMasterR.getEncoder().setPosition(0);
    driveSlaveL.getEncoder().setPosition(0);
    driveSlaveR.getEncoder().setPosition(0);
  }
  
  public void shiftPiston(boolean buttonAuto, boolean buttonAutoOff){
    
/*
    if (toggle && button) {  // Only execute once per Button push
      toggle = false;  // Prevents this section of code from being called again until the Button is released and re-pressed
      if (shifterFlag) {  // Decide which way to set the motor this time through (or use this as a motor value instead)
        shifterFlag = false;
        shifter.set(true);
      } else {
        shifterFlag= true;
        shifter.set(false);
      }
    } else if(button == false) { 
        toggle = true; // Button has been released, so this allows a re-press to activate the code above.
    }
*/if(buttonAuto){
    toggle = true;
  }
  if(buttonAutoOff){
    toggle = false;
  }
  if(toggle){
    if((Math.abs(Math.pow(Robot.robotContainer.leftSpeed(), 3)) >= .729 && Math.abs(driveMasterL.getEncoder().getVelocity()) >= 5000) && (Math.abs(Math.pow(Robot.robotContainer.rightSpeed(), 3)) >= .729 && Math.abs(driveMasterR.getEncoder().getVelocity()) >= 5000)){
      shifter.set(true);
    }
    else if(Math.abs(driveMasterL.getEncoder().getVelocity()) <= 3000 && Math.abs(driveMasterR.getEncoder().getVelocity()) <= 3000){
      shifter.set(false);
    }
  }
  else{
    shifter.set(false);
  }

/*
    if(buttonL && buttonH == false){
      shifter.set(true);
    }
    else if(buttonH && buttonL == false){
      shifter.set(false);
    }
*/      
  } 
  

}