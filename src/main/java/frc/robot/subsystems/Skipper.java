package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PIDTuning;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Arm;
import frc.robot.RobotContainer;

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

    // @Override
    // public void periodic(){


    //     double armPosition = robotContainer.m_arm.getArmPosition();
    //     double shooterVelo = robotContainer.m_shooter.getShooterVelo();

    //     if (armPosition < PIDTuning.Arm_Position_Threshold_UP 
    //     && shooterVelo < PIDTuning.Shooter_Velo_Threshold
    //     && armPosition > PIDTuning.Arm_Position_Threshold_Down) {
    //         setSkipperVelo(0.8);
    //     } else {
    //         if (shooterVelo > -200)
    //         setSkipperVelo(0);
    //     }
    // }
}
