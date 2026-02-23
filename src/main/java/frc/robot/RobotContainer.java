// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoShoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ShooterSystems.Hood;
import frc.robot.subsystems.ShooterSystems.Shooter;
import frc.robot.subsystems.ShooterSystems.ShotCalculator;
import frc.robot.subsystems.ShooterSystems.Turret;
public class RobotContainer 
{
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed, change multiplier to change max speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);      // Drive controller
    private final CommandXboxController operator = new CommandXboxController(1);    // Operator controller
    private final CommandXboxController test = new CommandXboxController(2);        // Test controller

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber s_climber = new Climber();
    public final Shooter s_shooter = new Shooter();
    public final Hood s_hood = new Hood();
    public final Turret s_turret = new Turret();
    public final Spindex s_spindex = new Spindex();
    public final BallElevator s_ballElevator = new BallElevator();
    public final ShotCalculator s_shotCalculator = new ShotCalculator(drivetrain);

    private final AutoShoot autoShootCommand = new AutoShoot(
            drivetrain,
            s_shooter,
            s_hood,
            s_turret,
            s_spindex,
            s_ballElevator,
            s_shotCalculator,
            () -> true, // TODO: replace with operator button/toggle when available
            () -> {
                // Sim helper: allow forcing score-enable without a physical LT input.
                if (RobotBase.isSimulation()
                        && SmartDashboard.getBoolean("AutoShoot/ForceScoreEnable", false)) {
                    return true;
                }
                // Hold LT to allow hub scoring on scoring side.
                return operator.getLeftTriggerAxis() > 0.5;
            },
            () -> true // TODO: replace/augment with explicit external scoring-window signal if desired
    );
    
    private final SendableChooser<Command> autoChooser; 

    public RobotContainer() 
    {
        SmartDashboard.putBoolean("AutoShoot/ForceScoreEnable", false);
        configureBindings();

        if (drivetrain.isAutoBuilderConfigured()) {
            autoChooser = AutoBuilder.buildAutoChooser();
        } else {
            autoChooser = new SendableChooser<>();
            autoChooser.setDefaultOption("No Auto (PathPlanner config missing)", null);
        }
        
        SmartDashboard.putData("Auto Mode", autoChooser); 
    }

    private void configureBindings() 
    {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand
        (
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest
            (() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        s_shooter.setDefaultCommand(autoShootCommand);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driver.a().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        driver.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric)); // Reset field centric heading to 0

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        //driver.leftBumper().whileTrue(); // Intake Left
        //driver.rightBumper().whileTrue(); // Intake Right

        //driver.rightTrigger().whileTrue(); // Manual Shoot

        drivetrain.registerTelemetry(logger::telemeterize);

        //TEST:
        test.a().onTrue(s_climber.runOnce(() -> s_climber.setHeightInches(Constants.Climber.MIN_HEIGHT_INCHES))); // a goes to minimum climber height
        test.y().onTrue(s_climber.runOnce(() -> s_climber.setHeightInches(Constants.Climber.MAX_HEIGHT_INCHES))); // y goes to max climber height
    }

    public Command getAutonomousCommand() 
    {
        return autoChooser.getSelected(); 
    }
}
