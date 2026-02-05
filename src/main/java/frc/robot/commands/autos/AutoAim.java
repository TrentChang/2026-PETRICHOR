package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;

public class AutoAim extends SequentialCommandGroup {
    public AutoAim(CommandSwerveDrivetrain drivetrain, CommandXboxController driveController){
        addCommands(drivetrain.autoAlignCommand(driveController));
    }
}