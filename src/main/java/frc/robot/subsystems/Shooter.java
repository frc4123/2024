package frc.robot.subsystems;

import frc.robot.Constants.SubsystemConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase{

    private CANSparkMax shooterLeader = new CANSparkMax(SubsystemConstants.Left_Shooter, MotorType.kBrushless);
    private CANSparkMax shooterFollower = new CANSparkMax(SubsystemConstants.Right_Shooter, MotorType.kBrushless);
    
    public Shooter(){
        shooterLeader.setOpenLoopRampRate(0.9);
        shooterLeader.setIdleMode(IdleMode.kBrake);
        shooterLeader.clearFaults();

        shooterFollower.setOpenLoopRampRate(0.9);
        shooterFollower.setIdleMode(IdleMode.kBrake);
        shooterFollower.clearFaults();
    }

    public void setShooterVelo(double velo){
        shooterLeader.set(velo);
        shooterFollower.follow(shooterLeader);
    }

    public double getShooterVelo() {
        return shooterLeader.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Velo", getShooterVelo());
    }
}
