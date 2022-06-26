package swerveLib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swerveLib.SwerveBase.SwerveConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FalconSwerveModule {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;

    private final SwerveConstants swerveConstants;

    /**
     * Constructs a SwerveModule for TalonFX motor controllers with a drive motor & turning motor.
     * 
     * @param driveMotorId - CAN ID for the drive motor.
     * @param turningMotorId - CAN ID for the turning motor.
     */
    public FalconSwerveModule(int driveMotorId, int turningMotorId, SwerveConstants swerveConstants) {
        driveMotor = new WPI_TalonFX(driveMotorId);
        turningMotor = new WPI_TalonFX(turningMotorId);

        this.swerveConstants = swerveConstants;

        swerveConstants.turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the curret state of the module.
     * 
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorVelocity(), new Rotation2d(turningMotor.getSelectedSensorPosition()));
    }

    /**
     * Sets the desired state for the module.
     * 
     * @param desiredState - Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningMotor.getSelectedSensorPosition()));

        final double driveOutput = swerveConstants.drivePidController.calculate(driveMotor.getSelectedSensorVelocity(), state.speedMetersPerSecond);
        final double driveFeedforward = swerveConstants.driveFeedforward.calculate(state.speedMetersPerSecond);

        final double turningOutput = swerveConstants.turningPidController.calculate(turningMotor.getSelectedSensorVelocity(), state.angle.getRadians());
        final double turningFeedforward = swerveConstants.turningFeedforward.calculate(swerveConstants.turningPidController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedforward);
        turningMotor.setVoltage(turningOutput + turningFeedforward);
    }

    /**
     * Set the raw velocity of the module.
     * @param driveSpeed - The velocity of the drive motor.
     * @param turningSpeed - The velocity of the turning motor.
     */
    public void setThrottle(double driveSpeed, double turningSpeed) {
        driveMotor.set(ControlMode.Velocity, driveSpeed);
        turningMotor.set(ControlMode.Velocity, turningSpeed);
    }

    /**
     * Set velocity of the drive & turning motors to 0.
     */
    public void stop() {
        driveMotor.set(ControlMode.Velocity, 0);
        turningMotor.set(ControlMode.Velocity, 0);
    }
}