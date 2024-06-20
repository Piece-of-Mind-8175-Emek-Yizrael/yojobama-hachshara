// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package pom.robot;

// import frc.robot.commands.*;
// import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.OpenIntakeCloseCommand;
import frc.robot.Commands.OpenOuttakeCloseCommand;
import frc.robot.POM_lib.Joysticks.PomXboxController;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private static RobotContainer m_robotContainer = new RobotContainer();

    ArmSubsystem m_armSubsystem = ArmSubsystem.getInstance();
    IntakeSubsystem m_intakeSubsystem = IntakeSubsystem.getInstance();
    DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();

    Trigger openIntakeCloseTrigger, openOuttakeCloseTrigger, closeArmTrigger, openArmTrigger, intakeOutTrigger, intakeInTrigger;

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
// The robot's subsystems

// Joysticks

    PomXboxController driverController = new PomXboxController(0);
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        openArmTrigger = new Trigger(driverController.a());
        closeArmTrigger = new Trigger(driverController.b());
        openIntakeCloseTrigger = new Trigger(driverController.x());
        openOuttakeCloseTrigger = new Trigger(driverController.y());
        intakeInTrigger = new Trigger(driverController.LB());
        intakeOutTrigger = new Trigger(driverController.RB());

        openArmTrigger.onTrue(m_armSubsystem.open(Constants.ArmConstants.ARM_POWER));
        closeArmTrigger.onTrue(m_armSubsystem.close(-Constants.ArmConstants.ARM_POWER));
        openIntakeCloseTrigger.onTrue(new OpenIntakeCloseCommand(m_intakeSubsystem, m_armSubsystem));
        openOuttakeCloseTrigger.onTrue(new OpenOuttakeCloseCommand(m_intakeSubsystem, m_armSubsystem));
        intakeInTrigger.onTrue(m_intakeSubsystem.intakeUntilIn(Constants.IntakeConstants.INTAKE_SPIN_POWER_IN));
        intakeOutTrigger.onTrue(m_intakeSubsystem.outtakeTime(1, Constants.IntakeConstants.INTAKE_SPIN_POWER_OUT));
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
        // Smartdashboard Subsystems

        // SmartDashboard Buttons
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_driveSubsystem.setDefaultCommand(m_driveSubsystem.driveCommand(() -> driverController.getLeftY(), () -> driverController.getRightX()));

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND

        // Configure autonomous sendable chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        SmartDashboard.putData("Auto Mode", m_chooser);
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
// Create some buttons
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS
    }

    public static RobotContainer getInstance() {
        return m_robotContainer;
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        return m_chooser.getSelected();
    }


}
