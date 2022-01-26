package frc.robot;

import edu.wpi.first.math.kinematics.*;

public class Constants {
    public static final int masterRightMotor = 1; // actual 1 test 2
    public static final int slaveRightMotor = 17; // actual 17 test 4
    public static final int masterLeftMotor = 14; // actual 14 test 1
    public static final int slaveLeftMotor = 15; // actual 15 test 3
    public static final int LClimb = 13;
    public static final int RClimb = 2;
    public static final int intakeRoller = 5;
    public static final int intakePanRoller = 7;
    public static final int intakeMotorTop = 6;
    public static final int intakeMotorBottom = 4;
    public static final int shooterMaster = 12;
    public static final int shooterSlave = 3;
    public static final int turretPort = 9;
    public static final int ballInternalCounterPort = 2;
    public static final int ballFeederCounterPort = 1;

    public static final double Kp = .025; // originally .025
    public static final double min_command = .05; //real bot .1
    public static final double distance_adjust = .05;
    public static final double ticksPerRev = 12.0; // Without Gear Reduction
    public static final double maxVelocity = 9.0; // meters per sec
    public static final double wheelDiameter = 0.1016;
    public static final double driveTrain_width = .4699;

    public static final double targetHeight = 89.75; //inches
    public static final double limelightAngle = 28.66; //on babybot (54.79), in degrees final bot: ?
    public static final double limelightHeight = 26; //inches

    public static final double desiredDistance = 60;
    public static final double kVolts = .461;
    public static final double kVSPM = 1.21; // volt seconds per meter
    public static final double kVSSPM = .425; // volt second squared per meter
    public static final double rSquared = .989;
    public static final double trackWidth = .599426775;
    public static final double kPDriveVelocity = 33.3;
    public static final double kD = 16.3;
    public static final double maxVolts = 28;

    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackWidth);
    // ideal distance 90 inches


    //Shooter Constants
    public static final double shootkP = .000625; //9e-20; 
    public static final double shootkI = 0; //7e-52;
    public static final double shootkD = 0; 
    public static final double shootkIz = 0; 
    public static final double shootkFF = 0.000220; //add 5 
    public static final double shootkMaxOutput = 1; 
    public static final double shootkMinOutput = -1;
    public static final double shootMaxRPM = 5700;
    public static final double shootsetPointConstant = -8000; // 3100 in auton
    public static double shootsetPointVariable = -8000; //-3100; competition speed should be -3100, change to -6000 for ice

    //Turret Constants
    public static final double turretkP = .007;
    public static final double turretkI = .00000000000001;
    public static final double turretkD = 0;
    public static final double turretClimbEncoderPos = 80000;

    //Climber Constants
    public static final double climberkP = .05;
    public static final double climberkI = 0;
    public static final double climberLCenter = -189;
    public static final double climberLLeft = -211;
    public static final double climberLRight = -211;
    public static final double climberRCenter = -180;
    public static final double climberRLeft = -193;
    public static final double climberRRight = -214;

    //Drive Constants 
    public static final double drivekP = 7; //.0312;
    public static final double drivekI = .0001;

    //Auton Constants
    public static final double driveGearingRatio = 18;
    public static final double kPAuto = 1.27;
    public static final double kVAuto = 2.5; //units: volts*seconds/meters; original: 4.69
    public static final double kAAuto = .36; //units: volts*seconds^2/meters; original: .179
    public static final double kDAuto = 0;
    public static final double kSAuto = 1.2; //units: volts; original kS = .12
    public static final double kGAuto = 0;
    public static final double kCosAuto = 0;
    public static final double rSquaredAuto = .998;

}
