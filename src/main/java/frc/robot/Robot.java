/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
//hi
//hi
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick; 
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Limelight;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean run = false;
  public double[] autoSpeeds;
  public boolean autoToggle = false;
  public boolean shootToggle = false;
  public int timer;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */


  public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public NetworkTableEntry tv = table.getEntry("tv");
  public NetworkTableEntry tx = table.getEntry("tx");
  public NetworkTableEntry ty = table.getEntry("ty");
  public NetworkTableEntry ta = table.getEntry("ta");
  public NetworkTableEntry ts = table.getEntry("ts");
  public NetworkTableEntry ledMode = table.getEntry("ledMode");
  public NetworkTableEntry stream = table.getEntry("stream");

  public static Limelight limelight = new Limelight();
  public static Joystick pad = new Joystick(0);
  public static Drive drive = new Drive();
  public static Climb climb = new Climb();
  public static Intake intake = new Intake();
  public static Turret turret = new Turret();
  public static RobotContainer robotContainer = new RobotContainer();
  public static SimpleAuto autonomous = new SimpleAuto(drive);
  public static Command align = new AutoAlign(limelight);
  public static Command tankDrive = new TankDrive(drive);
  //public static Command intakeForward = new RunIntakeForward(intake);
  //public static Command intakeBackward = new RunIntakeBackwards(intake);
  public static Command climbL = new ClimbExtendLatch(climb);
  //public static Command climbU = new ClimbUnLatch(climb);
  public static Command shift = new Shift(drive);
  public static Command shoot = new Shoot(intake);
  public static Command intakeSystem = new RunIntakeSystem(intake);
  public static Command turretShoot = new SetTurret(turret);
  //public static TrajectoryFollowing trajectoryFollowing = new TrajectoryFollowing();
  public static Compressor compressor;
  //public static Command auton = robotContainer.slalomRun(drive);
  public static Command deployIntakePistons = new DeployIntakePistons(intake);
  public static Timer time = new Timer();
  public static Command autoShoot = new AutoShoot(intake);
  public static Command autoAlign = new AutoAlign(limelight);
  public static Command uTurn = new RobotContainer().turn180(drive);
  public static Command autoStraight = new RobotContainer().autoStraight(drive);
  public static Command autoIntake = new AutoIntake(intake);
  public static Command resetOdom = new ResetOdom(drive, Math.PI);
  public static Command smallUTurn = new SmallUTurn(drive);

  public Command autonomouspt1 = new SequentialCommandGroup(autoAlign, autoShoot, uTurn);
  public Command autonomouspt2 = new ParallelRaceGroup(autoStraight, autoIntake);
  public Command auto = new SequentialCommandGroup(autonomouspt1, resetOdom, autonomouspt2, smallUTurn);
  public int autoTime;
  @Override
  public void robotInit() {

    autoTime = 0;
    drive.resetOdometry(new Pose2d(0, 0, new Rotation2d()));
    //drive.setDefaultCommand(tankDrive);
    

    //compressor = new Compressor(0);
    //compressor.clearAllPCMStickyFaults();
    //compressor.setClosedLoopControl(true);


  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    //System.out.println("Feeder Light value: " + intake.ballFeederCounter.get());
    //System.out.println("Internal Light value: " + intake.ballInternalCounter.get());
    limelight.setX(tx);
    limelight.setY(ty);
    limelight.setArea(ta);
    limelight.setV(tv);
    stream.setNumber(2);

    System.out.println("pose:" + drive.getPose());
    System.out.println("wheel speeds: " + drive.getWheelSpeeds());
    System.out.println("time: " + time.get());

    intake.count += 20;

    if(limelight.getY() < 7){
      limelight.shootsetPointVariable = 4100;
    } else if(limelight.getY() < 8){
      limelight.shootsetPointVariable = 4000;
    } else if(limelight.getY() < 11.5){
      limelight.shootsetPointVariable = 3900;
    } else {
      limelight.shootsetPointVariable = 3750;
    }

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    drive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    drive.getLeftMasterMotor().setSmartCurrentLimit(40);
    drive.getLeftSlaveMotor().setSmartCurrentLimit(40);
    drive.getRightMasterMotor().setSmartCurrentLimit(40);
    drive.getRightSlaveMotor().setSmartCurrentLimit(40);
    //drive.zeroReset();
    //drive.gyro.zeroYaw();
    //trajectoryFollowing.zeroHeading();
    //trajectoryFollowing.resetEncoders();
    

    //autonomous = robotContainer.getAutonomousCommand();
    /*
    autonomous = new SequentialCommandGroup(new AutoAlign(limelight), new AutoShoot(intake));
    if (autonomous != null) {
      autonomous.schedule();
    }
    ledMode.setNumber(3);
    */
    //m_autoSelected = kCustomAuto;

    /*autonomousCommand = robotContainer.getSimpleAuto(drive);
    autonomousCommand.initialize();

    if (autonomousCommand != null) {
        autonomousCommand.schedule();
    }*/
    auto.schedule();

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    autoTime += .02;
    CommandScheduler.getInstance().run();
    /*System.out.println("pose:" + drive.getPose());
    System.out.println("wheel speeds: " + drive.getWheelSpeeds());
    System.out.println("time: " + time.get());*/
    /*SmartDashboard.putNumber("X-Position: ", drive.getPose().getX());
    SmartDashboard.putNumber("Y-Position: ", drive.getPose().getY());
    SmartDashboard.putNumber("Left Wheel Speed: ", drive.getWheelSpeeds().leftMetersPerSecond);
    SmartDashboard.putNumber("Right Wheel Speed: ", drive.getWheelSpeeds().rightMetersPerSecond);
    SmartDashboard.putNumber("Gyro Heading: ", drive.getHeading());
    SmartDashboard.putNumber("Time: ", autoTime);*/


    if(auto.isFinished())
    {
      drive.SetPower(0, 0);
      autoTime = 0;

      drive.getLeftMasterMotor().setSmartCurrentLimit(30);
      drive.getLeftSlaveMotor().setSmartCurrentLimit(30);
      drive.getRightMasterMotor().setSmartCurrentLimit(30);
      drive.getRightSlaveMotor().setSmartCurrentLimit(30);
    }

    //timer += 20;
    //System.out.println(timer);
    /*
    if(timer < 2000){
      ledMode.setNumber(3);
      autoSpeeds = limelight.getSpeeds();
      drive.SetPower(autoSpeeds[0], autoSpeeds[1]);
    }
    else if(timer < 4000){
      ledMode.setNumber(1);
      intake.deployPistons(true, false);
      intake.autoShoot(true);
    }
    else if(timer < 6000){ 
      intake.autoShoot(false);
      drive.SetPower(.9, .9);
    }
    else if(timer > 6000){
      intake.autoShoot(false);
      drive.SetPower(0, 0);
    }
    */

    /*if(timer < 1000){
      ledMode.setNumber(3);
      autoSpeeds = limelight.getSpeeds();
      drive.SetPower(autoSpeeds[0], autoSpeeds[1]);
    }
    else if(timer < 3000){
      ledMode.setNumber(1);
      intake.autoShoot(true);
    }
    else if(timer < 3500){ 
      intake.autoShoot(false);
      drive.setDriveTrainRampRate(.5, .5);
      drive.autoPowerAuton(-12.1667, -15.8571, -12.1667, -15.8571);

    }
    else if(timer < 4500){
      drive.setDriveTrainRampRate(1, 1);
      drive.autoPowerAuton(-88.8371, 56.3101, -88.8371, 56.3101);
    }
    else if(timer < 5000){
      drive.zeroReset();
      drive.setDriveTrainRampRate(.5, .5);
      drive.autoPowerAuton(-29.1188, -29.1664, -29.1188, -29.1664);

    }
    else if (timer < 5250){
      drive.SetPower(0, 0);
    }
    else if(timer < 8250){
      intake.deployPistons(true, false);
      intake.runIntakeSystem(30, 0, false, false, false);
      drive.zeroReset();
      drive.setDriveTrainRampRate(3, 3);
      drive.autoPowerAuton(50, -50, 50, -50);
    }
    else if (timer < 8500){
      drive.SetPower(0, 0);
    }
    else if(timer < 10750){
      intake.deployPistons(false, false);
      intake.runIntakeSystem(0, 0, false, false, false);
      drive.zeroReset();
      drive.setDriveTrainRampRate(2, 2);
      drive.autoPowerAuton(-100, 100, -100, 100);
    }
    else if(timer < 11750){
      drive.zeroReset();
      drive.setDriveTrainRampRate(.5, .5);
      drive.autoPowerAuton(33.6902, 33.6902, 33.6902, 33.6902);
    }
    else if(timer < 12750){
      ledMode.setNumber(3);
      autoSpeeds = limelight.getSpeeds();
      drive.SetPower(autoSpeeds[0], autoSpeeds[1]);
    }
    else if(timer < 14750){
      ledMode.setNumber(1);
      intake.autoShoot(true);
    }
    else if(timer < 15000){
      intake.autoShoot(false);
    }*/

    //Part 1: MR: -15.8571 ML: -12.1667 SR: -15.8571 SL: -12.1667
    //Part 2: MR: 56.3101 ML: -88.8371 SR: 56.3101 SL: -88.8371
    //Part 3: MR: 21.3570 ML: -114.5485 SR: 21.3570 SL: -144.5485
    //Part 4: MR: -86.8369 ML: -5.6190 SR: -86.8369 SL: -5.6190
    //Part 5: MR: 62.9536 ML: -157.2334 SR: 62,9536 SL: -157.2334
    //Part 6: MR: 99.0029 ML: -110.4538 SR: 99.0029 SL: -110.4538
    //CommandScheduler.getInstance().run();
    //System.out.println(trajectoryFollowing.getPose());
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    
    /*System.out.println("ta:" + ta.getDouble(0.0));  
    System.out.println("log calculator thing" + (-4047.25 + (1699.79*(Math.floor(Math.log(limelight.area)*100)/100))));*/
    //System.out.println("Shoot set point" + limelight.shootsetPointVariable);
    //System.out.println(robotContainer.pad.getPOV());

    //moveTurret.schedule();
    tankDrive.schedule();
    //align.schedule();
    //climbL.schedule();
    climb.latch(Robot.robotContainer.startButton(), Robot.robotContainer.selectButton(), Robot.robotContainer.lTrigger(), Robot.robotContainer.rTrigger(), Robot.robotContainer.buttonB12Released(), Robot.robotContainer.buttonB14Released(), Robot.robotContainer.buttonB16Released());
    //climb.climberPos(robotContainer.buttonB12Released(), robotContainer.buttonB14Released(), robotContainer.buttonB16Released());
    //climbU.schedule();
    drive.shiftPiston(robotContainer.rBumper(), robotContainer.lBumper());

    robotContainer.aPad.whileHeld(align);
    //turret.turretLoop();
    //its me again
    turret.turretSetZero(robotContainer.xButton());
    turret.moveTurret(robotContainer.pad.getPOV());
    //turret.turretPosMove(robotContainer.yButtonReleased(), robotContainer.bButtonReleased());
    //System.out.println("Selected Sensor Pos: " + turret.getTurretSensorPos());
    //intake.shoot(robotContainer.yButtonM());
    /*
    if(robotContainer.yButtonM()){
      intake.shoot(robotContainer.yButtonM());
    }
    else{

    }
    */
    //shoot.schedule();
    intakeSystem.schedule();
    intake.deployPistons(Robot.robotContainer.rBumperM(), Robot.robotContainer.lBumperM());
    //intakeForward.schedule();
    //intakeBackward.schedule();
    //System.out.println("Distance: " + limelight.estimateDistance());
    //moveTurret.schedule();
    CommandScheduler.getInstance().run();
    
    if (robotContainer.aButton()){
      //limelight.shootsetPointVariable = 4100;//(3.16*Math.pow((limelight.getY() - 15), 2) - 21.25*(limelight.getY() - 15) + 4158.86); //-(-4047.25 + (1699.79*(Math.floor(Math.log(limelight.area)*100)/100)));
      if(limelight.getY() < 7){
        limelight.shootsetPointVariable = 4100;
      } else if(limelight.getY() < 8){
        limelight.shootsetPointVariable = 4000;
      } else if(limelight.getY() < 11.5){
        limelight.shootsetPointVariable = 3900;
      } else {
        limelight.shootsetPointVariable = 3750;
      }
      ledMode.setNumber(3);
    }
    else{
      ledMode.setNumber(3);
    }

    
    //intake.runInternalBelt(pad.getRawButtonPressed(3)); 
    //intake.runFeeder(pad.getRawButtonPressed(3)); //check this to make sure its the right button

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
