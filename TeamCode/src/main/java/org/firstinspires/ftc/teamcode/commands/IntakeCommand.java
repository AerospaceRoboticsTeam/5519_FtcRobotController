package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.activeIntake;

// Command to run intake at full power
public class IntakeCommand extends CommandBase {
    private final activeIntake intake;

    public IntakeCommand(activeIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}

// Command to outtake/eject
class OuttakeCommand extends CommandBase {
    private final activeIntake intake;

    public OuttakeCommand(activeIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.outtake();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}

// Command to move intake to specific angle
class MoveToAngleCommand extends CommandBase {
    private final activeIntake intake;
    private final double targetAngle;
    private final double tolerance;

    public MoveToAngleCommand(activeIntake intake, double targetAngle, double tolerance) {
        this.intake = intake;
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;
        addRequirements(intake);
    }

    public MoveToAngleCommand(activeIntake intake, double targetAngle) {
        this(intake, targetAngle, 2.0); // Default 2 degree tolerance
    }

    @Override
    public void initialize() {
        intake.setTargetAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return intake.isAtTarget(tolerance);
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            intake.holdCurrentPosition();
        } else {
            intake.stop();
        }
    }
}

// Command to stop intake
class StopIntakeCommand extends CommandBase {
    private final activeIntake intake;

    public StopIntakeCommand(activeIntake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}