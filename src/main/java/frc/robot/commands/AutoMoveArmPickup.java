package frc.robot.commands;
  
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Schlucker;

class Multithreading extends Thread {
    public void run () {
        try {

            System.out.println(
                Thread.currentThread().getId()
            );
        }
        catch (Exception e) {
            System.out.println("Failed");
        }
    }
}

public class AutoMoveArmPickup {
    public static void Main(String[] args) {
        int n=8;
        for (int i = 0; i < n; i++) {
            Multithreading object = new Multithreading();
            object.start();
        }
    }
}
   