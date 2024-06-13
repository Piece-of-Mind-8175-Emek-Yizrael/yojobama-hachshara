package frc.robot.Subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.BooleanSupplier;

public class IntakeSubsystem extends SubsystemBase
{
    CANSparkMax intakeMotor;

    public IntakeSubsystem() {intakeMotor = new CANSparkMax(Constants.IntakeConstants.INTAKE_PORT, CANSparkLowLevel.MotorType.kBrushless);}

    void spin(double speed) {intakeMotor.set(speed);}

    public Command intakeUntilIn(double speed)
        {return Commands.startEnd(() -> spin(speed),() -> spin(0),this)
                .until(() -> (intakeMotor.getOutputCurrent() > Constants.IntakeConstants.INTAKE_CURRENT_LIMIT));}
}
