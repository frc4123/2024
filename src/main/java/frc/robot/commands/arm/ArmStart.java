package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDTuning;
import frc.robot.subsystems.Arm;

public class ArmStart extends Command{
    Arm arm;

    public ArmStart(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setRadians(PIDTuning.Arm_RADIANS_START);
        arm.setPosition(PIDTuning.Arm_POSITION_START);
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    
}
