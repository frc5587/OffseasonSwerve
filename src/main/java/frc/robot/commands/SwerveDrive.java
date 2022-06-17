package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier xSupplier, ySupplier, omegaSupplier;
    private final BooleanSupplier fieldRelativeSupplier;

    public SwerveDrive(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier fieldRelativeSupplier) {
        this.drivetrain = drivetrain;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.swerveDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble(), fieldRelativeSupplier.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
