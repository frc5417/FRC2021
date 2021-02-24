package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.PIDController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;

import java.util.List;

//note: be very aware that the k constants in simplemotorfeedforward constructor could very likely be wrong
public class SimpleAuto extends SequentialCommandGroup{
    public SimpleAuto(Drive drivetrain){
        Subsystem d = drivetrain;
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
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

        //TODO: fix this
        RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drivetrain::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(Constants.kSAuto,
                                   Constants.kVAuto,
                                   Constants.kAAuto),
        Constants.kinematics,
        //problem: getwheelspeeds and tankdrivevolts need to be functional interfaces
        drivetrain.getWheelSpeeds(),
        drivetrain.getLeftMotor().getPIDController(),
        drivetrain.getRightMotor().getPIDController(),
        // RamseteCommand passes volts to the callback
        drivetrain.tankDriveVolts(12, 12), 
        drivetrain
        );

    }    
}