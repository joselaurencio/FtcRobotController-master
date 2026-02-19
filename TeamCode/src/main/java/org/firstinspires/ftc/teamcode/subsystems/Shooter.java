package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotorEx leftmotor;
    private final DcMotorEx rightmotor;
    private double targetRPM = 0;

    public Shooter(@NonNull HardwareMap hardwareMap) {
        leftmotor = hardwareMap.get(DcMotorEx.class, "left_launcher");
        leftmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightmotor = hardwareMap.get(DcMotorEx.class, "right_launcher");
        rightmotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightmotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setRPM(double rpm) {
        targetRPM = rpm;
        leftmotor.setVelocity(rpm * 28 / 60.0); // encoder ticks/sec (HD Hex)
        rightmotor.setVelocity(rpm * 28 / 60.0); // encoder ticks/sec (HD Hex)
    }

    public boolean atSpeed() {
        return Math.abs(getRPM() - targetRPM) < 100;
    }

    public double getRPM() {
        return leftmotor.getVelocity() * 60 / 28.0;
    }
}
