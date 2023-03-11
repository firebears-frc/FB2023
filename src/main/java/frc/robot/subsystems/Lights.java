package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    DigitalOutput togglePin;
    public Lights() {
        togglePin = new DigitalOutput(10);
        togglePin.set(false);        
    }

    public void setPin(boolean value) {
        togglePin.set(value);
    }
    public boolean getPin() {
        return togglePin.get();
    }
    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
