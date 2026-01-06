package org.firstinspires.ftc.teamcode.config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public final class robotConstants {
    private robotConstants() {}
    
    public static final class DriveTrain {
        private DriveTrain() {}
        public static final String LEFT_FRONT_MOTOR = "lfm";
        public static final String LEFT_BACK_MOTOR = "lbm";
        public static final String RIGHT_FRONT_MOTOR = "rfm";
        public static final String RIGHT_BACK_MOTOR = "rbm";
        public static final DcMotor.Direction LFM_DIRECTION = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction LBM_DIRECTION = DcMotor.Direction.REVERSE;
        public static final DcMotor.Direction RFM_DIRECTION = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction RBM_DIRECTION = DcMotor.Direction.FORWARD;
    }
    
    public static final class IMUDevice {
        private IMUDevice() {}
        public static final String NAME = "imu";
        private static final RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD;
        private static final RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        public static final RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);
        public static IMU.Parameters getParameters() { return new IMU.Parameters(orientationOnRobot); }
    }
    
    public static final class Intake {
        private Intake() {}
        public static final String MOTOR_NAME = "intake";
        public static final DcMotor.Direction DIRECTION = DcMotor.Direction.FORWARD;
        public static final double POWER = 0.9;  // Adjust as needed
    }
    
    public static final class Launcher {
        private Launcher() {}
        public static final String MOTOR_NAME = "launcher";
        public static final DcMotor.Direction DIRECTION = DcMotor.Direction.FORWARD;
        public static final double LAUNCH_POWER = 1.0;  // Full power for launching
        public static final double INTAKE_POWER = 0.5;  // Lower power for intaking
    }
}