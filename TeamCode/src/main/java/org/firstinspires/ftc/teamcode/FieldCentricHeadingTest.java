package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric drive test with auto-heading control.
 * Robot faces a specific point (e.g., a corner) when left trigger is held.
 */
@TeleOp(name = "Field-Centric Heading TEST", group = "IMU")
public class FieldCentricHeadingTest extends OpMode {

    Hardware robot = new Hardware();
    IMU imu;

    // For simplicity, we’ll simulate robot position.
    // If you have Road Runner or localization, plug in the real pose.
    double robotX = 24;  // starting X position on field (in inches)
    double robotY = 24;  // starting Y position on field (in inches)
    double heading = 0;

    // Target point on the field to face (bottom-left corner)
    double targetX = 0;
    double targetY = 0;

    double kP = 1.2;  // Proportional gain for heading correction

    @Override
    public void init() {
        robot.init(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("Initialized — hold left trigger to face (0,0)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;

        // Get robot’s current yaw from IMU
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Calculate desired heading (angle to target point)
        double targetAngle = Math.atan2(targetY - robotY, targetX - robotX);
        double headingError = AngleUnit.normalizeRadians(targetAngle - heading);

        // Compute rotation input
        double rotate = gamepad1.right_stick_x;

        // If left trigger held, override rotation to face target point
        if (gamepad1.left_trigger > 0.2) {
            rotate = kP * headingError;
        }

        // Drive field-relative
        driveFieldRelative(forward, right, rotate);

        // --- Telemetry ---
        telemetry.addData("Facing Mode", gamepad1.left_trigger > 0.2 ? "LOCKED" : "Manual");
        telemetry.addData("Target Angle (deg)", Math.toDegrees(targetAngle));
        telemetry.addData("Current Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        // Convert joystick direction to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Adjust by IMU heading
        theta = AngleUnit.normalizeRadians(theta - heading);

        // Convert back to Cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Standard mecanum mixing
        double frontLeftPower = newForward + newRight + rotate;
        double frontRightPower = newForward - newRight - rotate;
        double backRightPower = newForward + newRight - rotate;
        double backLeftPower = newForward - newRight + rotate;

        // Normalize powers
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        // Apply to motors
        robot.lF.setPower(frontLeftPower / max);
        robot.rF.setPower(frontRightPower / max);
        robot.lB.setPower(backLeftPower / max);
        robot.rB.setPower(backRightPower / max);
    }
}
