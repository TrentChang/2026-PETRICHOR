package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class autoAim extends SequentialCommandGroup {
    public autoAim(CommandSwerveDrivetrain drivetrain, CommandXboxController xboxController){
        addCommands(drivetrain.autoAlignCommand(xboxController));
    }
}