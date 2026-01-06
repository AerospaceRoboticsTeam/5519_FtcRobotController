package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.AdvancedDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.activeIntake;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.util.PedroPathingController;
import org.firstinspires.ftc.teamcode.util.Pose2d;

// ============================================
// TELEOP EXAMPLE
// ============================================

@TeleOp(name = "Main TeleOp", group = "Competition")
public class MainTeleOp extends CommandOpMode {

    // Panels configurables
    public static double configBoost = 1.0;
    public static double configSlowMode = 0.3;
    public static boolean configFieldCentric = true;

    private AdvancedDrivetrain drivetrain;
    private activeIntake intake;
    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    @Override
    public void initialize() {
        // Initialize subsystems
        PedroPathingController pathController = new PedroPathingController(hardwareMap);
        drivetrain = new AdvancedDrivetrain(hardwareMap, pathController);
        intake = new activeIntake(hardwareMap, "intakeMotor", 28.0, 0.01, 0, 0, 0);

        register(drivetrain, intake);

        // Initialize gamepads
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        // Set default command - driver controls drivetrain
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, driverGamepad));

        // Bind controls
        bindDriverControls();
        bindOperatorControls();

        telemetry.addLine("Robot Initialized");
        telemetry.update();
    }

    private void bindDriverControls() {
        // Reset heading with Start button
        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(new ResetHeadingCommand(drivetrain));

        // Slow mode with left trigger
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> drivetrain.setBoost(configSlowMode))
                .whenReleased(() -> drivetrain.setBoost(configBoost));

        // Emergency stop with back button
        new GamepadButton(driverGamepad, GamepadKeys.Button.BACK)
                .whenPressed(new StopDriveCommand(drivetrain));
    }

    private void bindOperatorControls() {
        // Intake controls
        new GamepadButton(operatorGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new IntakeCommand(intake));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new OuttakeCommand(intake));

        // Preset positions
        new GamepadButton(operatorGamepad, GamepadKeys.Button.A)
                .whenPressed(new MoveToAngleCommand(intake, 0.0));

        new GamepadButton(operatorGamepad, GamepadKeys.Button.B)
                .whenPressed(new MoveToAngleCommand(intake, 45.0));
    }

    @Override
    public void run() {
        // Sync Panels config
        drivetrain.setBoost(configBoost);

        super.run();

        // Optional telemetry
        telemetry.addData("Boost", "%.2f", configBoost);
        telemetry.update();
    }
}


// ============================================
// AUTONOMOUS EXAMPLE - Basic
// ============================================

@Autonomous(name = "Basic Auto", group = "Competition")
public class BasicAuto extends CommandOpMode {

    @Override
    public void initialize() {
        // Initialize subsystems
        PedroPathingController pathController = new PedroPathingController(hardwareMap);
        AdvancedDrivetrain drivetrain = new AdvancedDrivetrain(hardwareMap, pathController);
        activeIntake intake = new activeIntake(hardwareMap, "intakeMotor", 28.0, 0.01, 0, 0, 0);

        register(drivetrain, intake);

        // Simple autonomous sequence
        schedule(new SequentialCommandGroup(
                // Drive forward 24 inches
                new DriveStraightCommand(drivetrain, 24.0, 0.5),

                // Intake for 1 second
                new IntakeCommand(intake).withTimeout(1000),

                // Rotate 90 degrees
                new RotateToHeadingCommand(drivetrain, Math.toRadians(90)),

                // Drive to specific pose
                new DriveToPoseCommand(drivetrain, new Pose2d(48, 48, Math.toRadians(90)), 0.6),

                // Outtake
                new OuttakeCommand(intake).withTimeout(1000)
        ));
    }
}


// ============================================
// AUTONOMOUS EXAMPLE - Advanced with Parallel Actions
// ============================================

@Autonomous(name = "Advanced Auto", group = "Competition")
public class AdvancedAuto extends CommandOpMode {

    @Override
    public void initialize() {
        // Initialize subsystems
        PedroPathingController pathController = new PedroPathingController(hardwareMap);
        AdvancedDrivetrain drivetrain = new AdvancedDrivetrain(hardwareMap, pathController);
        activeIntake intake = new activeIntake(hardwareMap, "intakeMotor", 28.0, 0.01, 0, 0, 0);

        register(drivetrain, intake);

        // Complex autonomous with parallel actions
        schedule(new SequentialCommandGroup(
                // Drive to sample while preparing intake
                new ParallelCommandGroup(
                        new DriveToPoseCommand(drivetrain, new Pose2d(24, 24, 0), 0.6),
                        new MoveToAngleCommand(intake, 45.0)
                ),

                // Intake sample
                new IntakeCommand(intake).withTimeout(1500),

                // Drive to basket while stowing intake
                new ParallelCommandGroup(
                        new DriveToPoseCommand(drivetrain, new Pose2d(48, 48, Math.toRadians(90)), 0.8),
                        new MoveToAngleCommand(intake, 0.0)
                ),

                // Score
                new OuttakeCommand(intake).withTimeout(1000),

                // Park
                new DriveToPoseCommand(drivetrain, new Pose2d(60, 60, Math.toRadians(180)), 0.5)
        ));
    }
}


// ============================================
// AUTONOMOUS EXAMPLE - Using Generated Paths
// ============================================

@Autonomous(name = "Path Following Auto", group = "Competition")
public class PathFollowingAuto extends CommandOpMode {

    @Override
    public void initialize() {
        // Initialize subsystems
        PedroPathingController pathController = new PedroPathingController(hardwareMap);
        AdvancedDrivetrain drivetrain = new AdvancedDrivetrain(hardwareMap, pathController);
        activeIntake intake = new activeIntake(hardwareMap, "intakeMotor", 28.0, 0.01, 0, 0, 0);

        register(drivetrain, intake);

        // Load generated paths
        GeneratedPaths paths = new GeneratedPaths(pathController.getFollower());

        // Execute sequence using generated paths
        schedule(new SequentialCommandGroup(
                // Follow Path1 while intaking
                new ParallelCommandGroup(
                        new FollowPathCommand(pathController.getFollower(), paths.Path1),
                        new IntakeCommand(intake).withTimeout(2000)
                ),

                // Follow Path2 to scoring position
                new FollowPathCommand(pathController.getFollower(), paths.Path2),

                // Score
                new OuttakeCommand(intake).withTimeout(1000),

                // Return using simple commands
                new DriveStraightCommand(drivetrain, -12, 0.4)
        ));
    }
}


// ============================================
// UTILITY: Pre-built Action Sequences
// ============================================

/**
 * Common autonomous patterns as reusable methods
 */
class AutoActions {

    /**
     * Grab sample from ground
     */
    public static SequentialCommandGroup grabSample(
            AdvancedDrivetrain drivetrain,
            activeIntake intake,
            Pose2d samplePose
    ) {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new DriveToPoseCommand(drivetrain, samplePose, 0.6),
                        new MoveToAngleCommand(intake, 45.0)
                ),
                new IntakeCommand(intake).withTimeout(1500),
                new MoveToAngleCommand(intake, 0.0)
        );
    }

    /**
     * Score in high basket
     */
    public static SequentialCommandGroup scoreHighBasket(
            AdvancedDrivetrain drivetrain,
            activeIntake intake,
            Pose2d basketPose
    ) {
        return new SequentialCommandGroup(
                new DriveToPoseCommand(drivetrain, basketPose, 0.7),
                new MoveToAngleCommand(intake, 90.0),
                new OuttakeCommand(intake).withTimeout(1000),
                new MoveToAngleCommand(intake, 0.0)
        );
    }

    /**
     * Park in observation zone
     */
    public static DriveToPoseCommand park(
            AdvancedDrivetrain drivetrain,
            Pose2d parkPose
    ) {
        return new DriveToPoseCommand(drivetrain, parkPose, 0.5);
    }
}


// Usage of pre-built actions:
@Autonomous(name = "Action-Based Auto", group = "Competition")
class ActionBasedAuto extends CommandOpMode {
    @Override
    public void initialize() {
        PedroPathingController pathController = new PedroPathingController(hardwareMap);
        AdvancedDrivetrain drivetrain = new AdvancedDrivetrain(hardwareMap, pathController);
        activeIntake intake = new activeIntake(hardwareMap, "intakeMotor", 28.0, 0.01, 0, 0, 0);

        register(drivetrain, intake);

        schedule(new SequentialCommandGroup(
                AutoActions.grabSample(drivetrain, intake, new Pose2d(24, 24, 0)),
                AutoActions.scoreHighBasket(drivetrain, intake, new Pose2d(48, 48, Math.toRadians(90))),
                AutoActions.park(drivetrain, new Pose2d(60, 60, Math.toRadians(180)))
        ));
    }
}