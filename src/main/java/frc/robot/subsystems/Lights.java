package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        conePin = new DigitalOutput(11);
        cubePin = new DigitalOutput(12);
        conePin.set(false);
        cubePin.set(false);
        System.out.println("Setup lights");
        DriverStation.getAlliance();
        
    }

    public Alliance getTeamColor() {
        switch (DriverStation.getAlliance()) {
            case Blue:
                return Alliance.Blue;
            case Red:
            case Invalid:
            default:
            return Alliance.Red;
        }
    }

    public void showTeam() {
        System.out.println("Show team");
        switch(getTeamColor()) {
            case Blue:
                showBlue();
                break;
            case Red:
                showRed();
                break;
            case Invalid:
            default:
                break;
        }
    }

    private void showBlue() {
        conePin.set(false);
        cubePin.set(false);
    }
    private void showRed() {
        conePin.set(true);
        cubePin.set(true);
    }

    public void showCone() {
        System.out.println("Show cone");
        conePin.set(true);
        cubePin.set(false);
    }

    public void showCube() {
        System.out.println("Show cube");
        conePin.set(false);
        cubePin.set(true);
    }
    
    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
