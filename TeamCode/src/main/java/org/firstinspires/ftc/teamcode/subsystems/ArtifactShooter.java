package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

import dev.nextftc.core.subsystems.Subsystem;

public class ArtifactShooter implements Subsystem {
    Hardware robot;

    public ArtifactShooter(HardwareMap hw) {
        robot = new Hardware();
        robot.init(hw); // make sure to initialize your hardware here
    }

    public void setShooterVelocity(double velocity) {
        if (robot.shooterL == null || robot.shooterR == null) return;
        robot.shooterL.setVelocity(velocity);
        robot.shooterR.setVelocity(velocity);
    }

    public void stopShooter() {
        if (robot.shooterL == null || robot.shooterR == null) return;
        robot.shooterL.setPower(0);
        robot.shooterR.setPower(0);
    }

    public boolean isRunning() {
        return robot.shooterL != null && Math.abs(robot.shooterL.getVelocity()) > 50;
    }

    @Override
    public void periodic() {
        // Optional telemetry or monitoring
    }
}
