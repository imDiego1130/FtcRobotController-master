package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutoAimTest extends OpMode {
    Hardware robot = new Hardware();
    SampleMecanumDrive drive;

    boolean isFollowingTrajectory = false;
    Trajectory activeTrajectory;
    double fieldCentricOffset = 0;
    private boolean autoAimEnabled = false;

    IMU imu;

    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        robot.rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        drive.setPoseEstimate(new Pose2d(63, -63, Math.toRadians(-90))); // Adjust starting pose as needed

    }

    @Override
    public void loop() {
        // Update localization continuously
        drive.update();

        if (gamepad1.dpad_up) {
            imu.resetYaw();
            relocalizeHeading();
            //fieldCentricOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        if (gamepad1.dpad_right) {
            autoAimEnabled = true;
        }
        if (gamepad1.dpad_left) {
            autoAimEnabled = false;
        }
        if (autoAimEnabled && !isFollowingTrajectory) {
            //autoAimToGoal();  // robot keeps facing goal
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 0.9;
        double maxSpeed = 0.9;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        robot.lF.setPower(maxSpeed * (frontLeftPower / maxPower));
        robot.rF.setPower(maxSpeed * (frontRightPower / maxPower));
        robot.lB.setPower(maxSpeed * (backLeftPower / maxPower));
        robot.rB.setPower(maxSpeed * (backRightPower / maxPower));
    }
    public void relocalizeHeading() {
        // Reset IMU yaw to zero
        imu.resetYaw();

        // Get the current Road Runner pose
        Pose2d currentPose = drive.getPoseEstimate();

        // Read the IMU's current heading (should now be ~0 after reset)
        double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Keep x and y the same, only update heading
        drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));

        telemetry.addData("Heading re-localized", Math.toDegrees(imuHeading));
        telemetry.update();
    }
    // Function to follow a trajectory based on a target pose
    /*public void autoAimToGoal() {
        Pose2d pose = drive.getPoseEstimate();
        double currentx = pose.getX();
        double currenty = pose.getY();

        if (!isFollowingTrajectory) {
            //activeTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate()) // Use current pose
            activeTrajectory = drive.trajectorySequenceBuilder(pose)
                    .turn(Math.toRadians(-45) - pose.getHeading())
                    .build();
            drive.followTrajectoryAsync(activeTrajectory);
             isFollowingTrajectory = true;
        }
    }*/
    public void relocalizeToPose() {
        // Update the robot's pose estimate to (-54, -54, 45 degrees)
        drive.setPoseEstimate(new Pose2d(-63, -63, Math.toRadians(-90)));
        telemetry.addData("Relocalized to Pose:", new Pose2d(-63, -63, Math.toRadians(-90)));
        telemetry.update();
    }
}
