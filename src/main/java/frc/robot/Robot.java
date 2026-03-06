// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot 
{
    private PWM pwm;
    private static final int SERVO_MIN_US = 1000;
    private static final int SERVO_MAX_US = 2000;
    private static final int SERVO_CENTER_US = 1500;
    private static final int SERVO_NUDGE_US = 25;
    private int servoPulseUs = SERVO_CENTER_US;

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() 
    {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit()
    {
        // PWM port 6
        pwm = new PWM(6);

        // Set bounds to standard servo-style PWM
        pwm.setBoundsMicroseconds(
            2000, // max
            1500, // deadband high
            1500, // center
            1500, // deadband low
            1000  // min
        );

        pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
        pwm.setPulseTimeMicroseconds(servoPulseUs);
    }

    @Override
    public void robotPeriodic() 
    {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        m_robotContainer.processDashboardZeroRequests();
        m_robotContainer.processDashboardManualShooterRequests();
        m_robotContainer.printDiagnostics();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() 
    {
        pwm.setPulseTimeMicroseconds(servoPulseUs);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() 
    {
        m_robotContainer.stowIntakesAndStop();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) 
        {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() 
    {
        m_robotContainer.stowIntakesAndStop();
        if (m_autonomousCommand != null) 
        {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        int direction = m_robotContainer.consumeClimberServoNudgeDirection();
        if (direction != 0) {
            servoPulseUs = Math.max(SERVO_MIN_US, Math.min(SERVO_MAX_US, servoPulseUs + direction * SERVO_NUDGE_US));
            pwm.setPulseTimeMicroseconds(servoPulseUs);
        }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() 
    {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
