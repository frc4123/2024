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

    private CANSparkMax frontLeftArm = new CANSparkMax(SubsystemConstants.Front_Left_Arm, MotorType.kBrushless); // INVERTED
    private CANSparkMax frontRightArm = new CANSparkMax(SubsystemConstants.Front_Right_Arm, MotorType.kBrushless); // INVERTED
    private CANSparkMax backLeftArm = new CANSparkMax(SubsystemConstants.Back_Left_Arm, MotorType.kBrushless); 
    private CANSparkMax backRightArm = new CANSparkMax(SubsystemConstants.Back_Right_Arm, MotorType.kBrushless); 

    private SparkLimitSwitch m_reverseLimit = backRightArm.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

    private final ArmFeedforward m_feedforward = new ArmFeedforward(PIDTuning.Arm_FF_S, PIDTuning.Arm_FF_G, PIDTuning.Arm_FF_A);

    private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(PIDTuning.Arm_CONSTRAINTS_VELOCITY, PIDTuning.Arm_CONSTRAINTS_ACCELERATION);
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    private static double kDt = 0.02;
    private double setpoint = 0;

    public Arm(){
        frontLeftArm.setOpenLoopRampRate(0.8);
        frontLeftArm.setIdleMode(IdleMode.kBrake);
        frontLeftArm.clearFaults();

        frontRightArm.setOpenLoopRampRate(0.8);
        frontRightArm.setIdleMode(IdleMode.kBrake);
        frontRightArm.clearFaults();

        backLeftArm.setOpenLoopRampRate(0.8);
        backLeftArm.setIdleMode(IdleMode.kBrake);
        backLeftArm.clearFaults();

        backRightArm.setOpenLoopRampRate(0.8);
        backRightArm.setIdleMode(IdleMode.kBrake);
        backRightArm.clearFaults();

        backRightArm.getPIDController().setP(PIDTuning.Arm_PID_P);
        backRightArm.getPIDController().setI(PIDTuning.Arm_PID_I);
        backRightArm.getPIDController().setD(PIDTuning.Arm_PID_D);
    }

    public void setArmVelo(double velo){
        System.err.println("Calling setArmVelo when you should be using setPosition");
    }

    @Override
    public void periodic() {
        if (m_reverseLimit.isPressed()) { 
            backRightArm.getEncoder().setPosition(0);
        } // limit switch pressed zero's out arm

        internalSetPosition(setpoint);
        SmartDashboard.putBoolean("Where is arm?", !(m_reverseLimit.isPressed()));

    }

    public void enableBrakeMode(boolean isBrakeMode){
        frontLeftArm.setIdleMode(IdleMode.kBrake);
        frontRightArm.setIdleMode(IdleMode.kBrake);
        backLeftArm.setIdleMode(IdleMode.kBrake);
        backRightArm.setIdleMode(IdleMode.kBrake);
    }

    public void enableCoastMode(boolean isCoastMode){
        frontLeftArm.setIdleMode(IdleMode.kCoast);
        frontRightArm.setIdleMode(IdleMode.kCoast);
        backLeftArm.setIdleMode(IdleMode.kCoast);
        backRightArm.setIdleMode(IdleMode.kCoast);
    }

    public double positionToRadians(double encoderCounts) {
        // Convert counts to degrees
        double degrees = (double) encoderCounts * 360.0 / 4096;
      
        // Convert degrees to radians, considering gear ratio
        double radians = degrees * Math.PI / 180.0 * 88;
      
        // Return the radians value
        return radians;
    }

    public void setPosition(double position){
        setpoint = position;
    }

    public double getPosition() {
        return setpoint;
    }

    private void internalSetPosition(double position) {
        m_goal = new TrapezoidProfile.State(position, 0);
        new TrapezoidProfile.State(setpoint,0);
        TrapezoidProfile profile = new TrapezoidProfile(m_constraints);

        m_setpoint = profile.calculate(kDt,m_setpoint,m_goal);
      
        SmartDashboard.putNumber("Arm Position", backRightArm.getEncoder().getPosition());
        SmartDashboard.putNumber("Arm Radians", positionToRadians(backRightArm.getEncoder().getPosition()));
        SmartDashboard.putNumber("Arm Velocity", backRightArm.getEncoder().getVelocity());
  
        // Create a motion profile with the given maximum velocity and maximum
        // acceleration constraints for the next setpoint, the desired goal, and the
        // current setpoint.
    
        // Retrieve the profiled setpoint for the next timestep. This setpoint moves
        // toward the goal while obeying the constraints.
    
        // Send setpoint to offboard controller PID
        backRightArm.getPIDController().setReference(
            m_setpoint.position,
            CANSparkMax.ControlType.kPosition,
            0,
            m_feedforward.calculate(positionToRadians(m_setpoint.position),m_setpoint.velocity) / 12
        );

        frontRightArm.follow(backRightArm, true);
        backLeftArm.follow(backRightArm);
        frontLeftArm.follow(backRightArm, true);
    }
}
