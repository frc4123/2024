package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDTuning;
import frc.robot.subsystems.Arm;

public class ArmAutoSpeaker extends Command{
    Arm arm;

    public ArmAutoSpeaker(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setPosition(PIDTuning.Arm_POSITION_SHOOT + .3);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
