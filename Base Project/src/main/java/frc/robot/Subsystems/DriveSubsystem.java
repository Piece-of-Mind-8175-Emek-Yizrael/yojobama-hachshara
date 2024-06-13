package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase
{
    WPI_TalonSRX leftDriveTalon , rightDriveTalon;
    WPI_VictorSPX leftDriveVictor , rightDriveVictor;

    DifferentialDrive driveTrain;

    PigeonIMU imu;

    public DriveSubsystem()
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

    void drive(double forward, double turn) {driveTrain.arcadeDrive(forward , turn);}

    public Command driveCommand(DoubleSupplier forward, DoubleSupplier turn)
        {return Commands.run(() -> drive(forward.getAsDouble(),turn.getAsDouble()) , this);}

    public Command driveForwardTimeCommand(int time,double speed)
    {
        Timer timer = new Timer();
        timer.start();
        return Commands.startEnd(() -> drive(speed,0),() -> drive(0,0) ,this).until(() -> ((int)timer.get()*1000 > time));
    }

    public Command turnAngleCommand(double wantedAngle,double speed)
        {return Commands.startEnd(() -> drive(speed,0),
                () -> drive(0,0) ,
                this)
                .until(() -> Math.abs(wantedAngle -imu.getYaw()) < 5);}
}
