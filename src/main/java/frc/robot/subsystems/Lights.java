package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/*
 * Both low = Fire
 * One low = Cone / Cube
 * Two high = Rainbow
 */
public class Lights extends SubsystemBase {
    DigitalOutput conePin;
    DigitalOutput cubePin;
 
    public Lights() {
        conePin = new DigitalOutput(10);
        cubePin = new DigitalOutput(11);
        conePin.set(false);
        cubePin.set(false);
    }

    public void showFire() {
        conePin.set(false);
        cubePin.set(false);
    }

    public void showCone() {
        conePin.set(true);
        cubePin.set(false);
    }

    public void showCube() {
        conePin.set(false);
        cubePin.set(true);
    }
    public void showRainbow() {
        conePin.set(true);
        cubePin.set(true);
    }
    
    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
