package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.ClimberConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class Climber extends SubsystemBase{

    private CANSparkMax climberLeader = new CANSparkMax(SubsystemConstants.Right_Climber, MotorType.kBrushless);
    private CANSparkMax climberFollower = new CANSparkMax(SubsystemConstants.Left_Climber, MotorType.kBrushless); 

    public Climber(){
        climberLeader.setOpenLoopRampRate(0.2);
        climberLeader.setIdleMode(IdleMode.kBrake);
        climberLeader.clearFaults();

        climberFollower.setOpenLoopRampRate(0.2);
        climberFollower.setIdleMode(IdleMode.kBrake);
        climberFollower.clearFaults();
    }   

    public void setClimberVelo(double velo){
        climberLeader.set(velo);
        climberFollower.follow(climberLeader);
    }

    public void setRightClimberVelo(double velo){
        climberLeader.set(velo);
    }

    public void setLeftClimberVelo(double velo){
        climberFollower.set(velo);
    }

    public boolean isUnsafeVelocityUp() {
        return getClimberPosition() >= ClimberConstants.upperThreshold;
    }

    public boolean isUnsafeVelocityDown() {
        return getClimberPosition() <= ClimberConstants.lowerThreshold;
    }

    public double getClimberPosition() {
        return ((climberLeader.getEncoder().getPosition()) * -1); // to invert climbers, run climbers too high, they will eventually reverse
    }
}
