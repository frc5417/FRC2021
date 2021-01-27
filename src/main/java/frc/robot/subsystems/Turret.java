package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class Turret extends SubsystemBase{

    TalonSRX turret = new TalonSRX(Constants.turretPort);
    public int count;
    Timer timer = new Timer();

    public Turret(){
        turret.config_kP(0, Constants.turretkP);
        turret.config_kI(0, Constants.turretkI);
        turret.config_kD(0, Constants.turretkD);
        turret.configAllowableClosedloopError(0, 50);
        turret.configClosedloopRamp(1);

        turret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        turret.setNeutralMode(NeutralMode.Brake);
        timer.start();
        while(!timer.hasPeriodPassed(.5)){
          turret.set(ControlMode.PercentOutput, -1);
        }
        turret.set(ControlMode.PercentOutput, 0);
        turret.setSelectedSensorPosition(0);
        timer.stop();


    }

    public void moveTurret(int direction){
        if(direction == 90){
            turret.set(ControlMode.PercentOutput, .25);
        } else if(direction == 270){
            turret.set(ControlMode.PercentOutput, -.25);
        }
        else{
            turret.set(ControlMode.PercentOutput, 0);
        }
    }
    public void turretPosMove(boolean buttonClimb, boolean buttonShoot){
        turretLoop();
        if(buttonClimb && buttonShoot == false){
            count = 1;
            turret.set(ControlMode.Position, Constants.turretClimbEncoderPos);
        }
        else if(buttonShoot && buttonClimb == false){
            count = 0;
            turret.set(ControlMode.Position, 0);
        }
    }
    public void turretShoot(boolean button){
        if(button){
            //count = 0;
            turret.set(ControlMode.Position, 0);
            //System.out.println("Zero Code");
        }
    }
    public void turretSetZero(boolean button){
        if(button){
            turret.setSelectedSensorPosition(0);
        }
    }

    public void turretLoop(){
        switch(count){
            case 0:
                turret.set(ControlMode.Position, 0);
                break;
            case 1:
                turret.set(ControlMode.Position, Constants.turretClimbEncoderPos);
                break;
        }
    }
    public double getTurretSensorPos(){
        return turret.getSelectedSensorPosition();
    }

}