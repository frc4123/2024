// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import frc.robot.Constants.SubsystemConstants;

// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkLimitSwitch;

// public class ArmOpen extends SubsystemBase{

//     private CANSparkMax frontLeftArm = new CANSparkMax(SubsystemConstants.Front_Left_Arm, MotorType.kBrushless); // INVERTED
//     private CANSparkMax frontRightArm = new CANSparkMax(SubsystemConstants.Front_Right_Arm, MotorType.kBrushless); // INVERTED
//     private CANSparkMax backLeftArm = new CANSparkMax(SubsystemConstants.Back_Left_Arm, MotorType.kBrushless); 
//     private CANSparkMax backRightArm = new CANSparkMax(SubsystemConstants.Back_Right_Arm, MotorType.kBrushless); 

    
//     private SparkLimitSwitch m_reverseLimit = backRightArm.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

//     public ArmOpen(){
//         frontLeftArm.setOpenLoopRampRate(0.8);
//         frontLeftArm.setIdleMode(IdleMode.kBrake);
//         frontLeftArm.clearFaults();

//         frontRightArm.setOpenLoopRampRate(0.8);
//         frontRightArm.setIdleMode(IdleMode.kBrake);
//         frontRightArm.clearFaults();

//         backLeftArm.setOpenLoopRampRate(0.8);
//         backLeftArm.setIdleMode(IdleMode.kBrake);
//         backLeftArm.clearFaults();

//         backRightArm.setOpenLoopRampRate(0.8);
//         backRightArm.setIdleMode(IdleMode.kBrake);
//         backRightArm.clearFaults();

//     }   

//     public void setArmVelo(double velo){
//         backRightArm.set(velo);
//         frontLeftArm.follow(backRightArm, true);
//         backLeftArm.follow(backRightArm);
//         frontRightArm.follow(backRightArm, true);
//     }
// }
