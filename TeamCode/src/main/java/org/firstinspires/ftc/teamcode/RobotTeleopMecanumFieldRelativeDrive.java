/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GlobalConstants.BRAKE_OFF;
import static org.firstinspires.ftc.teamcode.GlobalConstants.BRAKE_ON;
import static org.firstinspires.ftc.teamcode.GlobalConstants.shooterSpeed_CLOSE;
import static org.firstinspires.ftc.teamcode.GlobalConstants.shooterSpeed_FAR;
import static org.firstinspires.ftc.teamcode.GlobalConstants.shooterSpeed_FEED;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactShooter;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "Blue Field-Centric", group = "BLUE")
//@Disabled
public class RobotTeleopMecanumFieldRelativeDrive extends OpMode {

    Hardware robot = new Hardware();
    ArtifactShooter shooter;
    AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    SampleMecanumDrive drive;

    boolean slowMode = false; // toggle state
    boolean prevStickPressed = false;
    boolean shooterActive = false;
    boolean prevBumperPressed = false;
    boolean prevButtonPressed = false;


    double speedMultiplier = slowMode ? 0.3 : 0.9;
    double ticksPerRev = 33.6; // REV Ultraplanetary 300 RPM motor
    double Feed = 0;
    double SpinDex = 0;
    double Shoot = 0;
    double Brake = 0;
    //double range = id20.ftcPose.range;  // distance in meters

    //double fieldCentricOffset = 0;
    private boolean autoAimEnabled = false;
    private boolean autoVelocityEnabled = false;

    //private static final double GOAL_X = -72;
    //private static final double GOAL_Y = 72;

    IMU imu;

    @Override
    public void init() {
        robot.init(hardwareMap);
        aprilTagWebcam.init(hardwareMap, telemetry);
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

        shooter = new ArtifactShooter(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-63, 9, Math.toRadians(90))); // Adjust starting pose as needed
    }
    @Override
    public void start(){
        Brake = BRAKE_OFF;
    }

    @Override
    public void loop() {
        drive.update();
        //update VisionPortal
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecificId(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
        //double range = id20.ftcPose.range;  // distance in meters
        //telemetry.addData("id20 String", id20.toString());

        if (gamepad1.right_stick_button && !prevStickPressed) {
            slowMode = !slowMode;
        }

        if (gamepad1.dpad_up) {
            imu.resetYaw();
           // relocalizeHeading();
            relocalizeToPose();
            //fieldCentricOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }
        /*if (gamepad1.dpad_down) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }*/

        // ----- Shooter Control -----
        /*if (gamepad1.right_bumper && !prevBumperPressed) {
            shooterActive = !shooterActive;
            if (shooterActive) {
                shooter.setShooterVelocity(shooterSpeed_FAR);
            } else {
                shooter.stopShooter();
            }
        }
        if (gamepad1.x && !prevButtonPressed) {
            shooterActive = !shooterActive;
            if (shooterActive) {
                shooter.setShooterVelocity(shooterSpeed_CLOSE);
            } else {
                shooter.stopShooter();
            }
        }*/
        prevBumperPressed = gamepad1.right_bumper;
        prevButtonPressed = gamepad1.x;
        // Adjust shooter speed with D-pad
        if (gamepad2.dpad_up) {
            Shoot += 50;
        }
        if (gamepad2.dpad_down){
            Shoot -= 50;
        }
        if (gamepad1.right_trigger > 0.01) {
            intake(1);
        } else if (gamepad1.left_trigger > 0.01) {
            intake(-1);
        } else {
            stopIntake();
        }
        if (gamepad1.right_bumper){
            feed(-1);
        }else if (gamepad1.left_bumper){
            feed(1);
            Shoot = shooterSpeed_FEED;
        }else {
            stopFeed();
        }
        if (id20 != null) {
            double yDistance = id20.ftcPose.y;
            double targetRPM = computeTargetRPM(yDistance);
            telemetry.addData("Target Shooter RPM", targetRPM);

            if (gamepad1.b) {
                Shoot = targetRPM;
                autoVelocityEnabled = true;
            }
            if (autoVelocityEnabled){
                Shoot = targetRPM;
            }
        }
        if (gamepad1.y){
            Shoot = shooterSpeed_FAR;
            autoVelocityEnabled = false;
        }
        else if (gamepad1.x) {
            Shoot = shooterSpeed_CLOSE;
            autoVelocityEnabled = false;
        }
        else if (gamepad1.a) {
            Shoot = 1000;
            autoVelocityEnabled = false;
        }
        /*if (gamepad1.dpad_left){
            Brake = BRAKE_OFF;
        }*/
        if (gamepad2.dpad_left){
            Brake -= 0.01;
        }
        if(gamepad2.dpad_right){
            Brake += 0.01;
        }
        double SPEED = robot.rB.getPower();
        if (SPEED == 0){
            Brake = BRAKE_ON;
        }else {
            Brake = BRAKE_OFF;
        }

       /* if (gamepad1.dpad_right) {
            // Goal-lock mode (hold right trigger)
            //autoAim();
            autoAimToGoal(gamepad1);
        } else if (gamepad1.dpad_left){
            // Normal field-centric control
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }*/
        if (gamepad1.dpad_right) {
            autoAimEnabled = true;
        }
        if (gamepad1.dpad_left) {
            autoAimEnabled = false;
        }
        if (autoAimEnabled) {
            autoAimToGoal(gamepad1);  // robot keeps facing goal
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


        shooter(Shoot);
        brakes(Brake);

        double shooterTicksPerSec = robot.shooterR.getVelocity();
        double shooterRpm = (shooterTicksPerSec * 60) / ticksPerRev;
        telemetry.addData("Slow Mode", slowMode ? "0.3" : "1.0");
        telemetry.addData("SHOOTER Power", robot.shooterL.getPower());
        telemetry.addData("RPM", shooterRpm);
        telemetry.addData("SPEED", robot.rB.getPower());
        telemetry.addData("Brake", robot.brake1.getPosition());
        telemetry.addData("Auto Aim Enabled", autoAimEnabled);
        //telemetry.addData("Raw range", id20.ftcPose.range);
        telemetry.update();
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

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = speedMultiplier;
        double maxSpeed = speedMultiplier;  // make this slower for outreaches

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
    public void shooter(double targetRpm) {
        // Convert RPM → ticks per second
        double targetTicksPerSecond = (targetRpm * ticksPerRev) / 60.0;

        robot.shooterR.setVelocity(targetTicksPerSecond);
        robot.shooterL.setVelocity(targetTicksPerSecond);
    }
    public void feed(double POWER){
        robot.feed.setPower(POWER);

        //robot.feed.setDirection(CRServo.Direction.REVERSE);
    }
    public void stopFeed(){
        robot.feed.setPower(0);
    }
    public void intake(double POWER){
        robot.intake.setPower(POWER);
    }
    public void stopIntake(){
        robot.intake.setPower(0);
    }
    /*public double calculateShooterRPMFromAprilTag(AprilTagDetection tag) {
        // y = 23.4x + 2062.9
        if (tag == null) {
            return 0; // no valid tag detected
        }

        //double rangeInches = tag.ftcPose.range * 39.3701; // convert meters → inches
        double rangeInches = (tag.ftcPose.range / 1000.0) * 39.3701; //converts mm -> in

        double targetRPM = 23.4 * rangeInches + 2062.9;   // apply your equation

        // clamp to safe range
        targetRPM = Math.max(2700, Math.min(4800, targetRPM));

        telemetry.addData("AprilTag Range (in)", rangeInches);
        telemetry.addData("Target Shooter RPM", targetRPM);

        return targetRPM;
    }*/

    // Function to compute target RPM based on Y distance in inches
    double computeTargetRPM(double yInches) {
        // y = 23.4x + 2062.9

        // Your derived linear relationship
        double targetRPM = 23.4 * (yInches - 7) + 2062.9;

        // Clamp to your tested range
        targetRPM = Math.max(2500, Math.min(5200, targetRPM));

        return targetRPM;
    }
    public void brakes(double POSITION){
        robot.brake1.setPosition(POSITION);
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
    public void relocalizeToPose() {
        drive.setPoseEstimate(new Pose2d(-63, -63, Math.toRadians(90)));
        telemetry.addData("Relocalized to Pose:", new Pose2d(-63, -63, Math.toRadians(90)));
        telemetry.update();
    }
    public void autoAimToGoal(Gamepad gamepad) {
        // --- Pose & Orientation ---
        Pose2d pose = drive.getPoseEstimate();
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Using IMU heading to stay consistent with your field-centric reference frame

        // --- Goal Coordinates ---
        double goalX = 72; //TODO: probably have to invert
        double goalY = 79;

        // --- Compute target heading toward the goal ---
        double deltaX = goalX - pose.getX();
        //double deltaY = goalY - pose.getY();
        double deltaY = pose.getY() - goalY;
        double targetHeading = Math.atan2(deltaY, deltaX);

        // --- Rotation control (auto-aim) ---
        double headingError = AngleUnit.normalizeRadians(targetHeading - currentHeading); //NORMAL HEADING
        // --- Rotation control (auto-aim) ---
        //double headingError = targetHeading - currentHeading; //CONVERTED HEADING 11/17/25 - LAST USE

        double kP = 0.085; // tune this value //0.075
        double turnPower = (Math.max(-1, Math.min(1, kP * headingError)));

        // --- Field-centric translation (same orientation as before) ---
        double y = -gamepad.left_stick_y; // forward/back
        double x = -gamepad.left_stick_x; // strafe

        // Rotate stick input based on current IMU heading (same as your regular drive)
        double rotX = x * Math.cos(-currentHeading) - y * Math.sin(-currentHeading);
        double rotY = x * Math.sin(-currentHeading) + y * Math.cos(-currentHeading);

        // --- Apply drive powers ---
        drive.setWeightedDrivePower(new Pose2d(rotY, rotX, turnPower));
    }

}
