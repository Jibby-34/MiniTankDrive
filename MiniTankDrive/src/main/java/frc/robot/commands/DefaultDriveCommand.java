package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends Command{
    DoubleSupplier leftSupplier;
    DoubleSupplier rightSupplier;
    DrivetrainSubsystem drivetrainSubsystem;

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier leftSupplier, DoubleSupplier rightSupplier) {
        this.leftSupplier = leftSupplier;
        this.rightSupplier = rightSupplier;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    /*@Override
    public void execute() {
        drivetrainSubsystem.drive(leftSupplier.getAsDouble(), rightSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(0, 0);
    }*/


    
}
