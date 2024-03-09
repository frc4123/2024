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
        arm.setPosition(PIDTuning.Arm_POSITION_PLACE);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
