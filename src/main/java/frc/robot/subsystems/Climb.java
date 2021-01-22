

package frc.robot.subsystems;

// climb code goes here

/*
1. Arm Starts lowered
2. Press Button to switch solenoid (piston will be removed) so arm can be raised
3. Turn motor so that ratchet starts spinning to lower arm (Holding down a button)
*/


// Reference example code for NEO Motor: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Motor%20Follower/src/main/java/frc/robot/Robot.java
// Reference for Solenoid: https://www.chiefdelphi.com/t/how-to-program-solenoid-and-compressor/101061/5
// Reference for SparkMAX deviceID: http://www.revrobotics.com/sparkmax-users-manual/


//This is the robot code for the CLIMBER

//Importing Packages For Solenoid And Spark(Neo) Motors

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climb extends SubsystemBase {
    
  	//Initializes Solenoids and compressor pistons

    CANSparkMax motorR = new CANSparkMax(Constants.RClimb, MotorType.kBrushless);
    CANSparkMax motorL = new CANSparkMax(Constants.LClimb, MotorType.kBrushless);


    boolean toggleR;
    boolean toggleL;
    int count;
    double climbPower;
    double climbRCenter;
    double climbRLeft;
    double climbRRight;
    double climbLCenter;
    double climbLLeft;
    double climbLRight;

    public Climb(){
      motorR.getPIDController().setP(Constants.climberkP);
      motorR.getPIDController().setI(Constants.climberkI);
      motorL.getPIDController().setP(Constants.climberkP);
      motorL.getPIDController().setI(Constants.climberkI);
      motorR.setInverted(false);
      motorL.setInverted(true);
      toggleR = false;
      toggleL = false;
      count = 0;
      climbPower = .5;
      motorR.setClosedLoopRampRate(1);
      motorL.setClosedLoopRampRate(1);
      climbRCenter = Constants.climberRCenter;
      climbRRight = Constants.climberRRight;
      climbRLeft = Constants.climberRLeft;
      climbLCenter = Constants.climberLCenter;
      climbLRight = Constants.climberLRight;
      climbLLeft = Constants.climberLLeft;
      motorR.getEncoder().setPosition(0);
      motorL.getEncoder().setPosition(0);
    }

    public void latch(boolean buttonDirectionUp, boolean buttonDirectionDown, double buttonTriggerLeft, double buttonTriggerRight, boolean buttonCenter, boolean buttonLeft, boolean buttonRight)
    {
      /*
      if(buttonLeftUp && buttonLeftDown == 0){
        motorL.set(-.5);
        System.out.println("latch function");
      }
      else if(buttonLeftDown != 0 && buttonLeftUp == false){
        motorL.set(.5);
      }
      else{
        motorL.set(0);
      }
      if(buttonRightUp && buttonRightDown == 0){
        motorR.set(-.5);
        System.out.println("latch function");
      }
      else if(buttonRightDown != 0 && buttonRightUp == false){
        motorR.set(.5);
      }
      else{
        motorR.set(0);
      }
      */
      //System.out.println("Motor Right Pos: " + motorR.getEncoder().getPosition());
      //System.out.println("Motor Left Pos: " + motorL.getEncoder().getPosition());
      if(buttonCenter && (buttonTriggerLeft == 0 && buttonTriggerRight == 0)){
        toggleR = true;
        toggleL = true;
        Robot.turret.turretPosMove(true, false);
        motorR.getPIDController().setReference(climbRCenter, ControlType.kPosition);
        motorL.getPIDController().setReference(climbLCenter, ControlType.kPosition);

      }
      else if(buttonLeft && (buttonTriggerLeft == 0 && buttonTriggerRight == 0)){
        toggleR = true;
        toggleL = true;
        Robot.turret.turretPosMove(true, false);
        motorR.getPIDController().setReference(climbRLeft, ControlType.kPosition);
        motorL.getPIDController().setReference(climbLLeft, ControlType.kPosition);

      }
      else if(buttonRight && (buttonTriggerLeft == 0 && buttonTriggerRight == 0)){
        toggleR = true;
        toggleL = true;
        Robot.turret.turretPosMove(true, false);
        motorR.getPIDController().setReference(climbRRight, ControlType.kPosition);
        motorL.getPIDController().setReference(climbLRight, ControlType.kPosition);

      }
      if(buttonDirectionUp){
        toggleR = false;
        toggleL = false;
        Robot.turret.turretPosMove(true, false);
        climbPower = .5;
      }
      if(buttonDirectionDown){
        toggleR = false;
        toggleL = false;
        Robot.turret.turretPosMove(true, false);
        climbPower = -.5;
      }
      if(buttonTriggerRight != 0){
        toggleR = false;
        Robot.turret.turretPosMove(true, false);
        motorR.set(climbPower);
      }
      else if (toggleR == false && buttonTriggerRight == 0){
        motorR.set(0);
      }
      if(buttonTriggerLeft != 0){
        toggleL = false;
        Robot.turret.turretPosMove(true, false);
        motorL.set(climbPower);
      }
      else if (toggleL == false && buttonTriggerLeft == 0){
        motorL.set(0);
      }
      
      

    }
    public void climberPos(boolean buttonCenter, boolean buttonLeft, boolean buttonRight){
      if(buttonCenter){
        Robot.turret.turretPosMove(true, false);
        motorR.getPIDController().setReference(climbRCenter, ControlType.kPosition);
        motorL.getPIDController().setReference(climbLCenter, ControlType.kPosition);
      }
      else if(buttonLeft){
        Robot.turret.turretPosMove(true, false);
        motorR.getPIDController().setReference(climbRLeft, ControlType.kPosition);
        motorL.getPIDController().setReference(climbLLeft, ControlType.kPosition);
      }
      else if(buttonRight){
        Robot.turret.turretPosMove(true, false);
        motorR.getPIDController().setReference(climbRRight, ControlType.kPosition);
        motorL.getPIDController().setReference(climbLRight, ControlType.kPosition);
      }
    }
    public void unLatch(double Power)
    {
      if(Power > 0){
        motorR.set(Power);
        motorL.set(Power);
        System.out.println("unlatch function");
      }
      else{
        motorR.set(0);
        motorL.set(0);
      }
    }



}
