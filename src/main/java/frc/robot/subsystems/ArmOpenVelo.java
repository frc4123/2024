// package frc.robot.subsystems;

// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.SubsystemConstants;

// public class ArmOpenVelo extends SubsystemBase{

//     private CANSparkMax frontLeftArm = new CANSparkMax(SubsystemConstants.Front_Left_Arm, MotorType.kBrushless);
//     private CANSparkMax frontRightArm = new CANSparkMax(SubsystemConstants.Front_Right_Arm, MotorType.kBrushless);
//     private CANSparkMax backLeftArm = new CANSparkMax(SubsystemConstants.Back_Left_Arm, MotorType.kBrushless);
//     private CANSparkMax backRightArm = new CANSparkMax(SubsystemConstants.Back_Right_Arm, MotorType.kBrushless);

//     public ArmOpenVelo(){

//         frontLeftArm.setOpenLoopRampRate(1);
//         frontLeftArm.setIdleMode(IdleMode.kBrake);
//         frontLeftArm.clearFaults();

//         frontRightArm.setOpenLoopRampRate(1);
//         frontRightArm.setIdleMode(IdleMode.kBrake);
//         frontRightArm.clearFaults();

//         backLeftArm.setOpenLoopRampRate(1);
//         backLeftArm.setIdleMode(IdleMode.kBrake);
//         backLeftArm.clearFaults();

//         backRightArm.setOpenLoopRampRate(1);
//         backRightArm.setIdleMode(IdleMode.kBrake);
//         backRightArm.clearFaults();

//         backLeftArm.setInverted(true);
//         backRightArm.setInverted(true);
//     }
    
//     public void setArmVelo(double velo){
//         //command below needs fixing
//         frontLeftArm.set(velo);
//         frontRightArm.follow(frontLeftArm);
//         backLeftArm.follow(frontLeftArm);
//         backRightArm.follow(frontLeftArm);
//     }
// }
