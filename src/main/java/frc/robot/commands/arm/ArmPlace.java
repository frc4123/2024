package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDTuning;
import frc.robot.subsystems.Arm;

public class ArmPlace extends Command{
    Arm arm;

    public ArmPlace(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // arm.setRadians(PIDTuning.Arm_RADIANS_PLACE);
        arm.setPosition(PIDTuning.Arm_POSITION_PLACE);
        System.out.println("Arm Place Called");
    }

    @Override
    public void end(boolean interrupted) {
    }
    
    
}
