package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase
{
    private static final int CONFIG_RETRIES = 5;
    
    private static final double STATOR_CURRENT_LIMIT_AMPS = 10.0;   // TODO - Adjust as necessary to prevent damage to the motor and mechanism - set low rn for testing purposes
    private static final double SUPPLY_CURRENT_LIMIT_AMPS = 10.0;   // TODO - Adjust as necessary to prevent damage to the motor and mechanism - set low rn for testing purposes
    private static final double MM_CRUISE_RPS = 5.0;
    private static final double MM_ACCEL_RPS2 = 5.0;
    private static final double SENSOR_TO_MECHANISM_RATIO = 78.33; // TOOD - Check this : Motor Gearbox = 20:1, 24T Gear -> 48T Gear = 2:1, 24T Pulley -> 47T Pulley = 47/24:1, Total Ratio = 20 * 2 * 47/24 = 78.33:1
    private static final double SLOT0_KS = 0.0;
    private static final double SLOT0_KV = 0.0;
    private static final double SLOT0_KP = 10.0;
    private static final double SLOT0_KI = 0.0;
    private static final double SLOT0_KD = 1.0;
    private static final double PEAK_FORWARD_VOLTS = 16.0;
    private static final double PEAK_REVERSE_VOLTS = -16.0;

    private double currentHeightSetPointInches = 0.0; // Store the current angle preset for alignment checks

    private final TalonFX climber_motor;
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public Climber()
    {
        climber_motor = new TalonFX(Constants.CAN_ID.CLIMBER, Constants.CAN_BUS.RIO);
        configureClimberMotor();
    }

    public void zeroEncoder() 
    {
        climber_motor.setPosition(0.0);
    }

    public void setHeightInches(double Height) 
    {
        climber_motor.setControl(motionMagicVoltage.withPosition(Height));
        currentHeightSetPointInches = Height; // Update the current angle preset for alignment checks
        SmartDashboard.putNumber("Climber Set Point", Height);
    }
    public void stop() 
    {
        climber_motor.stopMotor();
    }

    public double getHeightInches()
    {
        return climber_motor.getPosition().getValueAsDouble();
    }

    public double getHeightInchesSetpoint()
    {
        return currentHeightSetPointInches;
    }

    public boolean isAligned()
    {
        return Math.abs(getHeightInches() - currentHeightSetPointInches) <= Constants.Climber.HEIGHT_TOLERANCE_INCHES;
    }
    
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void configureClimberMotor() 
    {
        TalonFXConfiguration climberConfigs = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(STATOR_CURRENT_LIMIT_AMPS)
                        .withSupplyCurrentLimit(SUPPLY_CURRENT_LIMIT_AMPS)
                        .withStatorCurrentLimitEnable(true))
                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(SENSOR_TO_MECHANISM_RATIO))
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(MM_CRUISE_RPS))
                        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(MM_ACCEL_RPS2)));

        climberConfigs.Slot0.kS = SLOT0_KS;
        climberConfigs.Slot0.kV = SLOT0_KV;
        climberConfigs.Slot0.kP = SLOT0_KP;
        climberConfigs.Slot0.kI = SLOT0_KI;
        climberConfigs.Slot0.kD = SLOT0_KD;
        climberConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberConfigs.Voltage
                .withPeakForwardVoltage(Volts.of(PEAK_FORWARD_VOLTS))
                .withPeakReverseVoltage(Volts.of(PEAK_REVERSE_VOLTS));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < CONFIG_RETRIES; ++i) {
            status = climber_motor.getConfigurator().apply(climberConfigs);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not apply climber configs, error code: " + status);
        }
    }
}
