package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.GlobalConstants.shooterSpeed_FAR;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArtifactShooter;

@Disabled
@TeleOp(name = "Odo Field Centric", group = "Dead Wheels")
public class TeleOpFieldCentric extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ArtifactShooter shooter = new ArtifactShooter(hardwareMap);

        Hardware robot = new Hardware();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Load last known pose from Auto
        drive.setPoseEstimate(PoseStorage.currentPose);

        boolean slowMode = false; // toggle state
        boolean prevStickPressed = false;
        boolean shooterActive = false;
        boolean prevBumperPressed = false;


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Toggle slow mode when left stick is clicked
            if (gamepad1.left_stick_button && !prevStickPressed) {
                slowMode = !slowMode;
            }
            prevStickPressed = gamepad1.left_stick_button;

            // Field-centric translation input
            Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,   // forward/back
                    gamepad1.left_stick_x    // strafe
            ).rotated(-poseEstimate.getHeading());

            double speedMultiplier = slowMode ? 0.3 : 0.7;

            // Apply drive power with speed scaling
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * speedMultiplier,
                            input.getY() * speedMultiplier,
                            gamepad1.right_stick_x * speedMultiplier // rotation also scaled
                    )
            );

            //reset heading
            if (gamepad1.dpad_up) {
                drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), 0));
            }

            // ----- Shooter Control -----
            if (gamepad1.right_bumper && !prevBumperPressed) {
                shooterActive = !shooterActive;
                if (shooterActive) {
                    shooter.setShooterVelocity(shooterSpeed_FAR);
                } else {
                    shooter.stopShooter();
                }
            }
            prevBumperPressed = gamepad1.right_bumper;

            drive.update();

            telemetry.addData("X", poseEstimate.getX());
            telemetry.addData("Y", poseEstimate.getY());
            telemetry.addData("Heading (rad)", poseEstimate.getHeading());
            telemetry.addData("Slow Mode", slowMode ? "ON (0.3)" : "OFF (1.0)");
            telemetry.update();
        }
    }
    public static class PoseStorage {
        public static Pose2d currentPose = new Pose2d();
    }
}
