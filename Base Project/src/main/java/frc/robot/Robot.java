// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Robot.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.POM_lib.Joysticks.PomXboxController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    DifferentialDrive driveTrain;
    WPI_TalonSRX rightDriveTalon = new WPI_TalonSRX(Constants.DriveConstants.RIGHT_TALON);
    WPI_VictorSPX rightDriveVictor = new WPI_VictorSPX(Constants.DriveConstants.RIGHT_VICTOR);
    WPI_TalonSRX leftDriveTalon = new WPI_TalonSRX(Constants.DriveConstants.LEFT_TALON);
    WPI_VictorSPX leftDriveVictor = new WPI_VictorSPX(Constants.DriveConstants.LEFT_VICTOR);
    
    Boolean isAPreset = null;
    Boolean isYPreset = null;
    boolean isBPreset = false;
    boolean isXPreset = false;
    DigitalInput fold = new DigitalInput(Constants.FOLD_SWICH_PORT);
    DigitalInput ground = new DigitalInput(Constants.GROUND_SWICH_PORT);
    CANSparkMax motor = new CANSparkMax(Constants.INTAKE_PORT, MotorType.kBrushless);
    PomXboxController driverController = new PomXboxController(Constants.DRIVER_CONTROLLER_PORT);
    int diraction = 0;
    int lastDiraction = 0;
    private final CANSparkMax liftMotor = new CANSparkMax(Constants.LIFT_MOTOR_PORT, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private RelativeEncoder encoder = liftMotor.getEncoder();
    private ArmFeedforward ff = new ArmFeedforward(0, Constants.FF_KG , 0);

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */

    public void arcadeDrive(double y, double spin)
    {

    } 

    public double resistGravity(){
        return ff.calculate(encoder.getPosition(), 0);
    }
    
    @Override
    public void robotInit() {
        rightDriveVictor.follow(rightDriveTalon);
        leftDriveVictor.follow(leftDriveTalon);

        rightDriveTalon.setInverted(false);
        rightDriveVictor.setInverted(false);
        leftDriveTalon.setInverted(true);
        leftDriveVictor.setInverted(true);

        driveTrain = new DifferentialDrive(leftDriveTalon::set, rightDriveTalon::set);
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer.getInstance();
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        enableLiveWindowInTest(true);
        encoder.setPositionConversionFactor((1.0 / 50) * (16.0 / 42) * 2 * Math.PI);
    }

    /**
    * This function is called every robot packet, no matter the mode. Use this for items like
    * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("arm encoder", encoder.getPosition());
        SmartDashboard.putNumber("gravity resist", resistGravity());
        SmartDashboard.putBoolean("ground switch", !ground.get());
        SmartDashboard.putBoolean("fold switch", !fold.get());
        if(!fold.get())
        {
            encoder.setPosition(-0.323);
        }
    }

    public void drive() {driveTrain.arcadeDrive(driverController.getLeftY()/2, driverController.getRightX()/2);}

    public void doIntake()
    {
        if((isYPreset == null) && (isAPreset == null))
        {
            if(driverController.leftTrigger().getAsBoolean()) diraction=1;
            else if(driverController.rightTrigger().getAsBoolean()) diraction=-1;
            else diraction=0;
        }

        if(diraction != lastDiraction)
        {
            motor.set(Constants.INTAKE_SPIN_POWER * Math.signum(diraction));    
        }
    }

    public void doLift()
    {
        if(driverController.PovUp().getAsBoolean())
        {
            liftMotor.set(resistGravity());
            isAPreset = null;
            isYPreset = null;
            isXPreset = false;
            isBPreset = false;
        }
        if(driverController.aPressed().getAsBoolean())
        {
            isAPreset = true;
            isYPreset = null;
            isXPreset = false;
            isBPreset = false;
        }
        if(driverController.aReleased().getAsBoolean())
        {
            isAPreset = false;
            isYPreset = null;
            isXPreset = false;
            isBPreset = false;
        }
        if(driverController.yPressed().getAsBoolean())
        {
            isAPreset = null;
            isYPreset = true;
            isXPreset = false;
            isBPreset = false;    
        }
        if(driverController.yReleased().getAsBoolean())
        {
            isAPreset = null;
            isYPreset = false;
            isXPreset = false;
            isBPreset = false;    
        }
        if(driverController.bPressed().getAsBoolean()) 
        {
            isAPreset = null;
            isYPreset = null;
            isBPreset = !isBPreset;
            isXPreset = false;
        }
        else if(driverController.xPressed().getAsBoolean()) 
        {
            isAPreset = null;
            isYPreset = null;
            isXPreset = !isXPreset;
            isBPreset = false;
        }

        if(isXPreset && isBPreset)
        {
            isAPreset = null;
            isYPreset = null;
            isXPreset = false;
            isBPreset = false;
        }

        else if(isBPreset)
        {
            if(fold.get()) 
            {
                liftMotor.set(-Constants.ARM_POWER + resistGravity());
                isXPreset = false;
            }
            else
            {
                isBPreset = false;
                liftMotor.set(0);
            }
        }
        else if(isXPreset)
        {
            if(ground.get()) 
            {
                liftMotor.set(Constants.ARM_POWER + resistGravity());                 
                isBPreset = false;
            }
            else
            {
                isXPreset = false;
                liftMotor.set(0);
            }
        }

        else if(isYPreset != null)
        {
            if(!isYPreset)
            {
                diraction = 0;

                if(!fold.get()) 
                {
                    isYPreset = null;
                    liftMotor.set(0);
                }
                else liftMotor.set(-Constants.ARM_POWER + resistGravity());
            }
            else if(isYPreset)
            {
                if(!ground.get()) 
                {
                    liftMotor.set(0);
                    diraction = -1;
                }
                else liftMotor.set(Constants.ARM_POWER + resistGravity());
            }
        }

        else if(isAPreset != null)
        {
            if(!isAPreset)
            {
                diraction = 0;

                if(!fold.get()) 
                {
                    isAPreset = null;
                    liftMotor.set(0);
                }
                else liftMotor.set(-Constants.ARM_POWER + resistGravity());
            }
            else if(isAPreset)
            {
                if(!ground.get()) 
                {
                    liftMotor.set(0);
                    diraction = 1;
                }
                else liftMotor.set(Constants.ARM_POWER + resistGravity());
            }
        }
        
        else if(!fold.get() || !ground.get()) liftMotor.set(0);
        else liftMotor.set(resistGravity());
    }

    /**
    * This function is called once each time the robot enters Disabled mode.
    */
    @Override
    public void disabledInit() {
    //    System.err.println("I am a null and I am ok. I bully Uri most of the day");
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
    * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
    */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
    * This function is called periodically during autonomous.
    */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */

    @Override
    public void teleopPeriodic() {        
        drive();
        doLift();
        doIntake();

        lastDiraction = diraction;
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
    }

}
