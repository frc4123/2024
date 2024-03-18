package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogInput;


public class Sensor extends SubsystemBase{

    private AnalogInput sensor = new AnalogInput(0);
    
    public void sensor() {
        sensor.setOversampleBits(4);
        sensor.setAverageBits(4);

    }

    // public void Periodic() {
    //     SmartDashboard.putNumber("Sensor: ", sensor.getValue());
    // }
}
