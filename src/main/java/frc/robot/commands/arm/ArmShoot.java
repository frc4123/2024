package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDTuning;
import frc.robot.subsystems.Arm;

public class ArmShoot extends Command{
    Arm arm;

    public ArmShoot(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // arm.setRadians(PIDTuning.Arm_RADIANS_SHOOT);
        arm.setPosition(PIDTuning.Arm_POSITION_SHOOT);
        System.out.println("Arm Shoot called");
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    
}
