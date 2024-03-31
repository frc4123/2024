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

    private CANSparkMax topShooter = new CANSparkMax(SubsystemConstants.Top_Shooter, MotorType.kBrushless);
    private CANSparkMax bottomShooter = new CANSparkMax(SubsystemConstants.Bottom_Shooter, MotorType.kBrushless);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, PIDTuning.Shooter_FF_V, 0);

    private double targetVelo = 0;
    
    public ClosedShooter(){
        topShooter.setOpenLoopRampRate(0.2);
        topShooter.setIdleMode(IdleMode.kBrake);
        topShooter.clearFaults();

        bottomShooter.setOpenLoopRampRate(0.2);
        bottomShooter.setIdleMode(IdleMode.kBrake);
        bottomShooter.clearFaults();
        // τηισ ισ ωερυ ιμπορταντ

        topShooter.getPIDController().setP(PIDTuning.Shooter_PID_P);
        topShooter.getPIDController().setI(PIDTuning.Shooter_PID_I);
        topShooter.getPIDController().setD(PIDTuning.Shooter_PID_D);

        bottomShooter.getPIDController().setP(PIDTuning.Shooter_PID_P);
        bottomShooter.getPIDController().setI(PIDTuning.Shooter_PID_I);
        bottomShooter.getPIDController().setD(PIDTuning.Shooter_PID_D);
    }

    public void setShooterVelo(double targetVelo){
        topShooter.getPIDController().setReference(
            Constants.PIDTuning.Shooter_Target_Velo,
            CANSparkMax.ControlType.kVelocity,
            0,
            m_feedforward.calculate(targetVelo)
        );

        bottomShooter.getPIDController().setReference(
            Constants.PIDTuning.Shooter_Target_Velo,
            CANSparkMax.ControlType.kVelocity,
            0,
            m_feedforward.calculate(targetVelo)
        );
    }

    public void setTargetVelo(double velo) {
        targetVelo = velo;
    }

    public double getTopShooterVelo() {
        return topShooter.getEncoder().getVelocity();
    }
    
    public double getBottomShooterVelo() {
        return bottomShooter.getEncoder().getVelocity();
    }


    @Override
    public void periodic() {
        setTargetVelo(targetVelo);
        SmartDashboard.putNumber("Shooter Velo (Top)", getTopShooterVelo());
        SmartDashboard.putNumber("Shooter Velo (Bottom)", getBottomShooterVelo());
    }
}
