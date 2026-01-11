package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotorEx motor;
    private double targetRPM = 0;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
        motor.setVelocity(rpm * 28 / 60.0); // encoder ticks/sec (HD Hex)
    }

    public boolean atSpeed() {
        return Math.abs(getRPM() - targetRPM) < 100;
    }

    public double getRPM() {
        return motor.getVelocity() * 60 / 28.0;
    }
}
