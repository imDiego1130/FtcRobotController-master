package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.scale;

import static org.firstinspires.ftc.teamcode.GlobalConstants.FAST_SPEED;
import static org.firstinspires.ftc.teamcode.GlobalConstants.SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.GlobalConstants.shooterSpeed_CLOSE;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArtifactShooter;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Manual Modes", group="IMU")
public class ManualModes extends OpMode {
    Hardware robot = new Hardware();
    private ArtifactShooter shooter;
    private boolean shooterOn = false; // Tracks whether shooter motor is running

    double Shooter = 0;
    double MAX_SPEED = FAST_SPEED;
    double Feed = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);


        // Set PIDF coefficients for velocity control


        telemetry.addData("Say", "Hey Driver");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.left_stick_button) {
            MAX_SPEED = SLOW_SPEED;
        }
        if (gamepad1.right_stick_button) {
            MAX_SPEED = FAST_SPEED;
        }
        double Speed = -gamepad1.left_stick_y;
        double Turn = gamepad1.right_stick_x;
        double Strafe = -gamepad1.left_stick_x;
        holonomic(Speed, Turn, Strafe, MAX_SPEED);

        if (gamepad1.right_bumper) {

        }
        if (gamepad1.right_bumper && robot.shooterL.getVelocity() > shooterSpeed_CLOSE-150){
            feed();
        } else if (gamepad1.left_bumper) {
            stopFeed();
        }

        //feed(Feed);
        //transfer();


        telemetry.addData("SHOOTER Power", robot.shooterL.getPower());
        telemetry.update();
    }

    public void holonomic(double Speed, double Turn, double Strafe, double MAX_SPEED) {
        // Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        // Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
        double Magnitude = abs(Speed) + abs(Turn) + abs(Strafe);
        Magnitude = (Magnitude > 1) ? Magnitude : 1; //Set scaling to keep -1,+1 range

        robot.lF.setPower(scale((Speed + Turn - Strafe),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));

        if (robot.lB != null) {
            robot.lB.setPower(scale((Speed + Turn + Strafe),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
        robot.rF.setPower(scale((Speed - Turn + Strafe),
                -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        if (robot.rB != null) {
            robot.rB.setPower(scale((Speed - Turn - Strafe),
                    -Magnitude, +Magnitude, -MAX_SPEED, +MAX_SPEED));
        }
    }
    public void feed(){
        robot.feed.setPower(1);
    }
    public void stopFeed(){
        robot.feed.setPower(0);
    }

}
