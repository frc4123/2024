package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SubsystemConstants;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Skipper extends SubsystemBase{

    private CANSparkMax intakeSkipper = new CANSparkMax(SubsystemConstants.Intake_Skipper, MotorType.kBrushless);

    public Skipper(){
        intakeSkipper.setOpenLoopRampRate(0.2);
        intakeSkipper.setIdleMode(IdleMode.kBrake);
        intakeSkipper.clearFaults();
    }

    public void setSkipperVelo(double velo){
        intakeSkipper.set(velo);
    }
    
}
