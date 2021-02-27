package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.*;
import java.util.List;
import edu.wpi.first.wpilibj.kinematics.*; 
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.*;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import java.lang.Math;
import edu.wpi.first.wpilibj.Compressor;

public class RobotContainer{

    //public TrajectoryFollowing pathfollower;
    public Joystick pad;
    public Joystick padManipulator;
    public Joystick buttonBoard;
    public JoystickButton aPad;
    public JoystickButton aPadM;
    public JoystickButton xPadM;
    public JoystickButton bPadM;
    public JoystickButton yPadM;
    public JoystickButton rBumper;

    public RobotContainer(){
        //pathfollower = new TrajectoryFollowing();
        pad = new Joystick(0);
        padManipulator = new Joystick(1);
        buttonBoard = new Joystick(2);
        aPad = new JoystickButton(pad, 1);
        aPadM = new JoystickButton(padManipulator, 1);
        xPadM = new JoystickButton(padManipulator, 3);
        bPadM = new JoystickButton(padManipulator, 2);
        yPadM = new JoystickButton(padManipulator, 4);
        rBumper = new JoystickButton(pad, 6);

    }/*
    public Command getAutonomousCommand(){

        //pathfollower.resetOdometry(pathfollower.getPose());
        //pathfollower.zeroHeading();
/*
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.kVolts,
                                       Constants.kVSPM,
                                       Constants.kVSSPM),
            //pathfollower.getKinematics(),
            10);
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.maxVelocity, 3).setKinematics(pathfollower.getKinematics()).addConstraint(autoVoltageConstraint);

        Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(.5, 0)), new Pose2d(1, 0, new Rotation2d((0))), trajectoryConfig);
        
        RamseteCommand ramseteCommand = new RamseteCommand(traj, pathfollower::getPose, new RamseteController(2, 0.7), new SimpleMotorFeedforward(Constants.kVolts, Constants.kVSPM, Constants.kVSSPM), 
        pathfollower.getKinematics(), pathfollower::getSpeeds, new PIDController(Constants.kPDriveVelocity, 0, 0), new PIDController(Constants.kPDriveVelocity, 0, 0), pathfollower::tankDriveVolts, pathfollower);//add ref to differential drive object in trajectoryfollowing);

        return ramseteC 67ommand.andThen(() -> pathfollower.tankDriveVolts(0, 0));
        
    }
*/


    public double leftSpeed(){
        return pad.getRawAxis(1);
    }
    public double rightSpeed(){
        return pad.getRawAxis(5);
    }
    public double rTrigger(){
        return pad.getRawAxis(3);
    }
    public double lTrigger(){
        return pad.getRawAxis(2);
    }
    public double rTriggerM(){
        return padManipulator.getRawAxis(3);
    }
    public double lTriggerM(){
        return padManipulator.getRawAxis(2);
    }
    public boolean bButtonM(){
        return padManipulator.getRawButton(2);
    }
    public boolean bButton(){
        return pad.getRawButton(2);
    }
    public boolean bButtonReleased(){
        return pad.getRawButtonReleased(2);
    }
    public boolean lBumper(){
        return pad.getRawButton(5);
    }

    public boolean rBumper(){
        return pad.getRawButton(6);
    }
    public boolean lBumperM(){
        return padManipulator.getRawButton(5);
    }

    public boolean rBumperM(){
        return padManipulator.getRawButton(6);
    }

    public boolean yButton(){
        return pad.getRawButton(4);
    }
    public boolean yButtonReleased(){
        return pad.getRawButtonReleased(4);
    }
    public boolean xButtonM(){
        return padManipulator.getRawButton(3);
    }
    public boolean startButton(){
        return pad.getRawButton(7);
    }
    public boolean selectButton(){
        return pad.getRawButton(8);
    }
    public boolean aButtonM(){
        return padManipulator.getRawButton(1);
    }
    public boolean aButton(){
        return pad.getRawButton(1);
    }

    public double turretSpeed(){
        return padManipulator.getRawAxis(5); // right side
    }

    public boolean yButtonM(){
        return padManipulator.getRawButton(4);
    }

    public boolean xButton(){
        return pad.getRawButton(3);
    }

    public boolean buttonB2(){
        return buttonBoard.getRawButton(2);
    }
    public boolean buttonB4(){
        return buttonBoard.getRawButton(4);
    }
    public boolean buttonB6(){
        return buttonBoard.getRawButton(6);
    }
    public boolean buttonB8(){
        return buttonBoard.getRawButton(8);
    }
    public boolean buttonB10(){
        return buttonBoard.getRawButton(10);
    }
    public boolean buttonB12(){
        return buttonBoard.getRawButton(12);
    }
    public boolean buttonB14(){
        return buttonBoard.getRawButton(14);
    }
    public boolean buttonB16(){
        return buttonBoard.getRawButton(16);
    }
    public boolean buttonB12Released(){
        return buttonBoard.getRawButtonReleased(12);
    }
    public boolean buttonB14Released(){
        return buttonBoard.getRawButtonReleased(14);
    }
    public boolean buttonB16Released(){
        return buttonBoard.getRawButtonReleased(16);
    }

    public Command getSimpleAuto(Drive drivetrain){
            var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.kSAuto, Constants.kVAuto, Constants.kAAuto),
            Constants.kinematics,
            10);
    
            //trajectoryconfig parameters are in meters. may need to fix numbers
            TrajectoryConfig config = 
            new TrajectoryConfig(9, 9).setKinematics(Constants.kinematics).addConstraint(autoVoltageConstraint);
    
            Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                    new Translation2d(1, 0),
                    new Translation2d(2, 0)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config
            );
            //System.out.println(exampleTrajectory);
    
            drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
    
            RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            drivetrain::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.kSAuto,
                                       Constants.kVAuto,
                                       Constants.kAAuto),
            Constants.kinematics,
            //problem: getwheelspeeds and tankdrivevolts need to be functional interfaces
            drivetrain::getWheelSpeeds,
            /*drivetrain.getLeftMotor().getPIDController(),
            drivetrain.getRightMotor().getPIDController(),*/
            new PIDController(Constants.kPAuto, Constants.drivekI, 0),
            new PIDController(Constants.kPAuto, Constants.drivekI, 0),
            // RamseteCommand passes volts to the callback
            drivetrain::tankDriveVolts, 
            drivetrain
            );
            System.out.println("you have gone past the ramsete command");
    
            // Reset odometry to the starting pose of the trajectory.
            drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
    
            // uh i really do not know if this will work
            return ramseteCommand;//.andThen(() -> drivetrain.tankDriveVolts(0, 0));
    

    }

}

