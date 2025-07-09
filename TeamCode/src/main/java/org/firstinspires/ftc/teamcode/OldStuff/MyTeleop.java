package org.firstinspires.ftc.teamcode.OldStuff;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MyTeleop extends LinearOpMode {
    DcMotor motor;
    Servo servo;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(gamepad1.left_stick_y);
            if (gamepad1.a) {
                servo.setPosition(0.2);
            }
            else if (gamepad1.b) {
                servo.setPosition(0.8);
            }
        }
    }
}
