package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.PIDTuning;
import frc.robot.Constants.SubsystemConstants;
import frc.robot.commands.shooter.AutoShooter;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClosedShooter extends SubsystemBase{

    // THIS IS USING VELOCITY/RPM POSITION CONTROL

    private CANSparkMax shooterLeader = new CANSparkMax(SubsystemConstants.Left_Shooter, MotorType.kBrushless);
    private CANSparkMax shooterFollower = new CANSparkMax(SubsystemConstants.Right_Shooter, MotorType.kBrushless);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, PIDTuning.Shooter_FF_V, 0);

    private double targetVelo = 0;
    
    public ClosedShooter(){
        shooterLeader.setOpenLoopRampRate(0.2);
        shooterLeader.setIdleMode(IdleMode.kBrake);
        shooterLeader.clearFaults();

        shooterFollower.setOpenLoopRampRate(0.2);
        shooterFollower.setIdleMode(IdleMode.kBrake);
        shooterFollower.clearFaults();
        // τηισ ισ ωερυ ιμπορταντ

        shooterLeader.getPIDController().setP(PIDTuning.Shooter_PID_P);
        shooterLeader.getPIDController().setI(PIDTuning.Shooter_PID_I);
        shooterLeader.getPIDController().setD(PIDTuning.Shooter_PID_D);
    }

    public void setShooterVelo(double targetVelo){
        shooterLeader.getPIDController().setReference(
            Constants.PIDTuning.Shooter_Target_Velo,
            CANSparkMax.ControlType.kVelocity,
            0,
            m_feedforward.calculate(targetVelo)
        );

        shooterFollower.follow(shooterLeader);
    }

    public void setTargetVelo(double velo) {
        targetVelo = velo;
    }

    public double getShooterVelo() {
        return shooterLeader.getEncoder().getVelocity();
    }


    @Override
    public void periodic() {
        setTargetVelo(targetVelo);
        SmartDashboard.putNumber("Shooter Velo", getShooterVelo());
    }
}
