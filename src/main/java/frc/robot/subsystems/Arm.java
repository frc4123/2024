package frc.robot.subsystems;

import frc.robot.Constants.SubsystemConstants;
import frc.robot.Constants.PIDTuning;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

    private CANSparkMax frontLeftArm = new CANSparkMax(SubsystemConstants.Front_Left_Arm, MotorType.kBrushless);
    private CANSparkMax frontRightArm = new CANSparkMax(SubsystemConstants.Front_Right_Arm, MotorType.kBrushless); 
    private CANSparkMax backLeftArm = new CANSparkMax(SubsystemConstants.Back_Left_Arm, MotorType.kBrushless);
    private CANSparkMax backRightArm = new CANSparkMax(SubsystemConstants.Back_Right_Arm, MotorType.kBrushless); 

    private SparkLimitSwitch m_forwardLimit = frontLeftArm.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(PIDTuning.Arm_FF_S, 0, PIDTuning.Arm_FF_A);

    private final TrapezoidProfile.Constraints m_constraints =
    new TrapezoidProfile.Constraints(PIDTuning.Arm_CONSTRAINTS_VELOCITY, PIDTuning.Arm_CONSTRAINTS_ACCELERATION);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private static double kDt = 0.02;
    private double setpoint = 0;
    private double positionRadians = 0;

    public Arm(){
        frontLeftArm.setOpenLoopRampRate(0.7);
        frontLeftArm.setIdleMode(IdleMode.kBrake);
        frontLeftArm.clearFaults();

        frontLeftArm.getPIDController().setP(PIDTuning.Arm_PID_P);
        frontLeftArm.getPIDController().setI(PIDTuning.Arm_PID_I);
        frontLeftArm.getPIDController().setD(PIDTuning.Arm_PID_D);
    }

    public void setArmVelo(double velo){
        System.err.println("Calling setArmVelo when you should be using setPosition");
        // wrist.set(velo);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      internalSetPosition(setpoint, positionRadians);
      SmartDashboard.putBoolean("Where is arm?", !(m_forwardLimit.isPressed()));
    }

    public void setPosition(double position){
        setpoint = position;
    }

    public void setRadians(double radians){
        positionRadians = radians;
    }

    public double getPosition() {
        return setpoint;
    }

    private void internalSetPosition(double position, double radians) {
        m_goal = new TrapezoidProfile.State(position, 0);
        new TrapezoidProfile.State(setpoint,0);
        TrapezoidProfile profile = new TrapezoidProfile(m_constraints);

        m_setpoint = profile.calculate(kDt,m_setpoint,m_goal);
      
  

        SmartDashboard.putNumber("Arm Position", frontLeftArm.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Velocity", frontLeftArm.getEncoder().getVelocity());
  
        // Create a motion profile with the given maximum velocity and maximum
        // acceleration constraints for the next setpoint, the desired goal, and the
        // current setpoint.
    
        // Retrieve the profiled setpoint for the next timestep. This setpoint moves
        // toward the goal while obeying the constraints.
    
        // Send setpoint to offboard controller PID
        frontLeftArm.getPIDController().setReference(
            m_setpoint.position,
            CANSparkMax.ControlType.kPosition,
            0,
            m_feedforward.calculate(positionRadians,m_setpoint.velocity) / 12

        );

        frontRightArm.follow(frontLeftArm);
        backLeftArm.follow(frontLeftArm);
        backRightArm.follow(frontLeftArm);
            
    }
}
