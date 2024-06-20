package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class OpenOuttakeCloseCommand extends Command
{
    IntakeSubsystem intakeSubsystem;
    ArmSubsystem armSubsystem;

    public OpenOuttakeCloseCommand(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem)
    {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize()
    {
        armSubsystem.open(Constants.ArmConstants.ARM_POWER)
                .andThen(intakeSubsystem.outtake(Constants.IntakeConstants.INTAKE_SPIN_POWER_OUT)
                        .andThen(new WaitCommand(1)
                            .andThen(armSubsystem.close(-Constants.ArmConstants.ARM_POWER))));
    }
}
