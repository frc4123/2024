package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.SubsystemConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private CANSparkMax shooterLeader = new CANSparkMax(SubsystemConstants.Left_Shooter, MotorType.kBrushless);
    private CANSparkMax shooterFollower = new CANSparkMax(SubsystemConstants.Right_Shooter, MotorType.kBrushless);

    public Shooter(){
        shooterLeader.setOpenLoopRampRate(0.7);
        shooterLeader.setIdleMode(IdleMode.kBrake);
        shooterLeader.clearFaults();

        shooterFollower.setOpenLoopRampRate(0.7);
        shooterFollower.setIdleMode(IdleMode.kBrake);
        shooterFollower.clearFaults();
    }

    public void setShooterVelo(double velo){
        //command below needs fixing
        shooterLeader.set(velo);
        shooterFollower.follow(shooterLeader);
    }
    
}
