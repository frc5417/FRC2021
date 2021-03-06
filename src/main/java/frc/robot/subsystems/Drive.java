/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.TankDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;

import java.lang.Math;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.*;

/**
 * Add your docs here.
 */
public class Drive extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.s
  
  public CANSparkMax driveSlaveL = new CANSparkMax(Constants.slaveLeftMotor, MotorType.kBrushless);
  public CANSparkMax driveMasterR = new CANSparkMax(Constants.masterRightMotor, MotorType.kBrushless);
  public CANSparkMax driveSlaveR = new CANSparkMax(Constants.slaveRightMotor, MotorType.kBrushless);
  public CANSparkMax driveMasterL = new CANSparkMax(Constants.masterLeftMotor, MotorType.kBrushless);

  public CANEncoder neoEncoderL = driveMasterL.getEncoder();
  public CANEncoder neoEncoderR = driveMasterR.getEncoder();

  public AHRS gyro = new AHRS(Port.kUSB);

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

  private DifferentialDrive drive;
  private DifferentialDriveOdometry driveOdom;


  public Drive(){
/*
    driveMasterRight.setNeutralMode(NeutralMode.Coast);
    driveMasterLeft.setNeutralMode(NeutralMode.Coast);
    driveSlaveRight.setNeutralMode(NeutralMode.Coast);
    driveSlaveLeft.setNeutralMode(NeutralMode.Coast);
*/

    turretMotor.setNeutralMode(NeutralMode.Coast);

    neoEncoderL.setPosition(0);
    neoEncoderR.setPosition(0);

    //according to api, neo encoder track distance based on full rotations, not individual encoder ticks.
    //units in meters
    neoEncoderL.setPositionConversionFactor(Math.PI*.1524 / Constants.driveGearingRatio);
    neoEncoderR.setPositionConversionFactor(Math.PI*.1524 / Constants.driveGearingRatio);

    neoEncoderL.setVelocityConversionFactor(Math.PI*.1524 / Constants.driveGearingRatio);
    neoEncoderR.setVelocityConversionFactor(Math.PI*.1524 / Constants.driveGearingRatio);

    drive = new DifferentialDrive(driveMasterL, driveMasterR);
    //driveMasterL->setSafetyEnabled(false);
    //drive.setSafetyEnabled(false);

    driveOdom = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));




    driveMasterL.getPIDController().setP(Constants.drivekP);
    driveMasterR.getPIDController().setP(Constants.drivekP);
    driveSlaveL.getPIDController().setP(Constants.drivekP);
    driveSlaveR.getPIDController().setP(Constants.drivekP);
    driveMasterL.getPIDController().setI(Constants.drivekI);
    driveMasterR.getPIDController().setI(Constants.drivekI);
    driveSlaveL.getPIDController().setI(Constants.drivekI);
    driveSlaveR.getPIDController().setI(Constants.drivekI);

    /*driveMasterL.getEncoder().setPosition(0);
    driveMasterR.getEncoder().setPosition(0);
    driveSlaveL.getEncoder().setPosition(0);
    driveSlaveR.getEncoder().setPosition(0);*/
    
    shifter = new Solenoid(0);
    zeroReset();
    gyro.zeroYaw();

  }

  public CANSparkMax getLeftMotor(){
    return driveMasterL;
  }

  public CANSparkMax getRightMotor(){
    return driveMasterR;
  }
/*
  public double[] getDifDrive(){
    DifferentialDriveWheelSpeeds difDrive = new DifferentialDriveWheelSpeeds(neoEncoderL.getVelocity(), neoEncoderR.getVelocity());
    double leftSpeeds = difDrive.leftMetersPerSecond;
    double rightSpeeds = difDrive.rightMetersPerSecond;
    double[] wheelSpeeds = {leftSpeeds, rightSpeeds};
    return wheelSpeeds;
  }
*/
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    System.out.println("wheel speeds: " + new DifferentialDriveWheelSpeeds(neoEncoderL.getVelocity(), -neoEncoderR.getVelocity()));
    return new DifferentialDriveWheelSpeeds(neoEncoderL.getVelocity(), -neoEncoderR.getVelocity());
  }
  
  public Pose2d getPose(){
    //System.out.println("poseMeters:"+ driveOdom.getPoseMeters()); 
    return driveOdom.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    zeroReset();
    gyro.zeroYaw();
    driveOdom.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    System.out.println("leftvolts: " + leftVolts + "rightvolts: " + rightVolts);
    driveMasterL.set(leftVolts / RobotController.getBatteryVoltage());
    driveMasterR.set(-rightVolts / RobotController.getBatteryVoltage());
    drive.feed();
  }

  public double getAverageEncoderDistance(){
    return(neoEncoderL.getPosition() + neoEncoderR.getPosition()) / 2.0;
  }

  public double getLeftDistance(){
    return neoEncoderL.getPosition();
  }

  public double getRightDistance(){
    return neoEncoderR.getPosition();
  }

  public CANEncoder getLeftNeoEncoder(){
    return neoEncoderL;
  }

  public CANEncoder getRightNeoEncoder(){
    return neoEncoderR;
  }

  public void setMaxOutput(double maxOutput){
    drive.setMaxOutput(maxOutput);
  }

  public void SetPower(double leftPower, double rightPower){
    
    /*driveMasterL.setIdleMode(IdleMode.kCoast);
    driveMasterR.setIdleMode(IdleMode.kCoast);
    driveSlaveL.setIdleMode(IdleMode.kCoast);
    driveSlaveR.setIdleMode(IdleMode.kCoast);*/

    driveSlaveL.follow(driveMasterL);
    driveSlaveR.follow(driveMasterR);
    //if (!((leftPower < .1)&&(leftPower > -.1) || (rightPower < .1)&&(rightPower > -.1))){
    /*  
    driveMasterLeft.set(ControlMode.PercentOutput, -leftPower);
    driveMasterRight.set(ControlMode.PercentOutput, -rightPower);
    driveSlaveLeft.set(ControlMode.PercentOutput, leftPower);
    driveSlaveRight.set(ControlMode.PercentOutput, -rightPower);
    */

    driveMasterL.set(-Math.pow(leftPower, 3));
    driveMasterR.set(Math.pow(rightPower, 3)); //inversions for bowser
    //driveSlaveL.set(-Math.pow(leftPower, 3));
    //driveSlaveR.set(Math.pow(rightPower, 3));
    //System.out.println("Left Speed: " + driveMasterL.getEncoder().getVelocity());
    //System.out.println("Right Speed: " + driveMasterR.getEncoder().getVelocity());
    System.out.println("distance traveled teleop: right: " + getRightDistance() + " left: " + getLeftDistance());
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

  public double getHeading(){
    return Math.IEEEremainder(gyro.getAngle(), 360);
    //return -gyro.getYaw();
  }

  @Override
  public void periodic(){
    driveOdom.update(Rotation2d.fromDegrees(getHeading()), neoEncoderL.getPosition(), -neoEncoderR.getPosition());
  }
}