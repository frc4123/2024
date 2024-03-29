package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.PIDTuning;

public class ArmIntake extends Command{
    Arm arm;

    public ArmIntake(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.setPosition(PIDTuning.Arm_POSITION_INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
