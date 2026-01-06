package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.robotConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@TeleOp(name = "Testing TeleOp", group = "Test")
public class Testing_TeleOp extends LinearOpMode {
    // Drive train power variables
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    
    // Subsystems
    private IntakeSubsystem intake;
    private LauncherSubsystem launcher;

    public void drive() {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        frontLeftPower = axial + lateral + yaw;
        frontRightPower = axial - lateral - yaw;
        backLeftPower = axial - lateral + yaw;
        backRightPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }
    }

    @Override
    public void runOpMode() {
        // Initialize subsystems
        intake = new IntakeSubsystem(hardwareMap);
        launcher = new LauncherSubsystem(hardwareMap);
        
        // Initialize drive motors
        DcMotor frontLeft = hardwareMap.dcMotor.get(robotConstants.DriveTrain.LEFT_FRONT_MOTOR);
        DcMotor frontRight = hardwareMap.dcMotor.get(robotConstants.DriveTrain.RIGHT_FRONT_MOTOR);
        DcMotor backLeft = hardwareMap.dcMotor.get(robotConstants.DriveTrain.LEFT_BACK_MOTOR);
        DcMotor backRight = hardwareMap.dcMotor.get(robotConstants.DriveTrain.RIGHT_BACK_MOTOR);
        
        // Set motor directions from constants
        frontLeft.setDirection(robotConstants.DriveTrain.LFM_DIRECTION);
        frontRight.setDirection(robotConstants.DriveTrain.RFM_DIRECTION);
        backLeft.setDirection(robotConstants.DriveTrain.LBM_DIRECTION);
        backRight.setDirection(robotConstants.DriveTrain.RBM_DIRECTION);
        
        // Set zero power behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Handle drive train
            drive();
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            
            // Handle intake (right trigger to intake, left trigger to outtake)
            if (gamepad2.right_trigger > 0.1) {
                intake.setPower(robotConstants.Intake.POWER);
            } else if (gamepad2.left_trigger > 0.1) {
                intake.setPower(-robotConstants.Intake.POWER);
            } else {
                intake.stop();
            }
            
            // Handle launcher (Y to launch, A to reverse for intake, B to stop)
            if (gamepad2.y) {
                launcher.launch();
                telemetry.addData("Launcher", "Launching");
            } else if (gamepad2.a) {
                launcher.intake();
                telemetry.addData("Launcher", "Intaking");
            } else if (gamepad2.b) {
                launcher.stop();
                telemetry.addData("Launcher", "Stopped");
            }
            
            // Add telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Intake", "Power: %.2f", 
                gamepad2.right_trigger > 0.1 ? robotConstants.Intake.POWER : 
                (gamepad2.left_trigger > 0.1 ? -robotConstants.Intake.POWER : 0));
            telemetry.update();
        }
    }
}
