package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.scale;
import static org.firstinspires.ftc.teamcode.GlobalConstants.FAST_SPEED;
import static org.firstinspires.ftc.teamcode.GlobalConstants.SLOW_SPEED;
import static org.firstinspires.ftc.teamcode.GlobalConstants.shooterSpeed_FAR;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArtifactShooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp Drive", group="IMU")
public class TeleOpDriver extends OpMode {
    Hardware robot = new Hardware();
    private ArtifactShooter shooter;
    private boolean shooterOn = false; // Tracks whether shooter motor is running

    double Shooter = 0;
    double MAX_SPEED = FAST_SPEED;

    double kP = 10.0;
    double kI = 0.0;
    double kD = 0.0;
    double kF = 11.7;
    double targetRpm = 4800;
    double ticksPerRev = 28.0; // REV Ultraplanetary 6000 RPM motor

    @Override
    public void init() {
        robot.init(hardwareMap);
        shooter = new ArtifactShooter(hardwareMap);

        // Set PIDF coefficients for velocity control
        robot.shooterR.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        robot.shooterL.setVelocityPIDFCoefficients(kP, kI, kD, kF);

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
            Shooter = shooterSpeed_FAR;
        } else if (gamepad1.left_bumper ) {
            Shooter = 0;
        }
        if (gamepad1.dpad_up){
            Shooter += 100;
        }
        if (gamepad1.dpad_down){
            Shooter -= 100;
        }

        if (gamepad1.a) {
            Shooter = 2000;
        } else if (gamepad1.b ) {
            Shooter = 0;
        }

        // Run shooter at target RPM
        shooter(Shooter);

        // Calculate live velocity and RPM for telemetry
        double shooterTicksPerSec = robot.shooterR.getVelocity();
        double shooterRpm = (shooterTicksPerSec * 60) / ticksPerRev;

        telemetry.addData("SHOOTER Power", robot.shooterL.getPower());
        telemetry.addData("Shooter Velocity (ticks/sec)", shooterTicksPerSec);
        telemetry.addData("Shooter Velocity (RPM)", shooterRpm);
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

    public void shooter(double targetRpm) {
        // Convert RPM → ticks per second
        double targetTicksPerSecond = (targetRpm * ticksPerRev) / 60.0;

        robot.shooterR.setVelocity(targetTicksPerSecond);
        robot.shooterL.setVelocity(targetTicksPerSecond);
    }
}
