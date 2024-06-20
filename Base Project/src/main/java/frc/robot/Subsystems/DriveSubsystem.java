package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase
{
    public static DriveSubsystem instance;

    public static DriveSubsystem getInstance()
        {   if (instance == null) instance = new DriveSubsystem();
            return instance;}

    WPI_TalonSRX leftDriveTalon , rightDriveTalon;
    WPI_VictorSPX leftDriveVictor , rightDriveVictor;

    DifferentialDrive driveTrain;

    PigeonIMU imu;

    private DriveSubsystem()
    {
        rightDriveTalon = new WPI_TalonSRX(Constants.DriveConstants.RIGHT_TALON);
        rightDriveVictor = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_VICTOR);
        leftDriveTalon = new WPI_TalonSRX(Constants.DriveConstants.LEFT_TALON);
        leftDriveVictor = new WPI_VictorSPX(Constants.DriveConstants.LEFT_VICTOR);

        rightDriveVictor.follow(rightDriveTalon);
        leftDriveVictor.follow(leftDriveTalon);

        rightDriveTalon.setInverted(true);
        rightDriveVictor.setInverted(true);
        leftDriveTalon.setInverted(false);
        leftDriveVictor.setInverted(false);

        driveTrain = new DifferentialDrive(leftDriveTalon::set, rightDriveTalon::set);

        imu = new PigeonIMU(Constants.DriveConstants.PigeonIMU);
    }

    public double getAngle() {return imu.getYaw();}

    void drive(double forward, double turn) {driveTrain.arcadeDrive(forward , turn);}

    public Command driveCommand(DoubleSupplier forward, DoubleSupplier turn)
        {return Commands.run(() -> drive(forward.getAsDouble(),turn.getAsDouble()) , instance);}

    public Command driveForwardTimeCommand(double seconds,double speed)
        {return Commands.startEnd(() -> drive(speed,0),() -> drive(0,0) ,instance).deadlineWith(new WaitCommand(seconds));}

    public Command turnAngleCommand(double wantedAngle,double speed)
        {return Commands.startEnd(() -> drive(speed,0),
                () -> drive(0,0) ,
                instance)
                .until(() -> Math.abs(wantedAngle -imu.getYaw()) < 5);}
}
