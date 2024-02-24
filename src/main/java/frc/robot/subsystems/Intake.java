package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase{

    private CANSparkMax intakeLeader = new CANSparkMax(SubsystemConstants.Ground_Intake, MotorType.kBrushless);

    public Intake(){
        intakeLeader.setOpenLoopRampRate(-0.7);
        intakeLeader.setIdleMode(IdleMode.kBrake);
        intakeLeader.clearFaults();
    }
    
    public void setIntakeVelo(double velo){
        //command below needs fixing
        intakeLeader.set(velo);
    }
}
