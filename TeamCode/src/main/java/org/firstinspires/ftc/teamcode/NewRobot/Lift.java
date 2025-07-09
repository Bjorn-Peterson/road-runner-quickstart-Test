package org.firstinspires.ftc.teamcode.NewRobot;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Lift {
    public enum LiftState {
        START,
        LIFT,
        LIFTED,
        DOWN
    }

    public enum TeleState {
        START,
        LOW,
        SPECIMEN,
        MANUAL,
        UP
    }

    TeleState teleState = TeleState.START;
    LiftState liftState = LiftState.START;
    PIDController controller;
    PIDController lController;
    public static double p = 0.025, i = 0, d = 0.001;
    public static double lp = 0.025, li  = 0, ld = .00001;
    public static double f = 0.021;
    public static int target;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);
    double closed = .61;
    double open = .42;
    double specClosed = .54;
    double backSpec = .72;
    double midPos = .54;
    double specPos = .77;
    double backMid = .8;



     DcMotorEx lift;
     DcMotorEx lift2;
    public Servo claw;
    public Servo deliveryS;
    public Servo hExtend;
    public DigitalChannel liftTouch;
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime deliveryTimer = new ElapsedTime();
    OpMode theOpMode;
    double countsPerInch;
    DigitalChannel dBeam;



    public Lift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        lController = new PIDController(lp,li,ld);
        controller = new PIDController(p, i, d);
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        theOpMode = opMode;
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
        hExtend = hardwareMap.get(Servo.class, "hExtend");
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        liftTouch = hardwareMap.get(DigitalChannel.class, "lTouch");
        liftTouch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void teleLift() {

        switch (teleState) {
            case START:
                if (theOpMode.gamepad1.dpad_right) {
                    teleState = TeleState.UP;
                }
                if (theOpMode.gamepad1.right_trigger > 0.1 || theOpMode.gamepad1.left_trigger > 0.1) {
                    teleState = TeleState.MANUAL;
                }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case MANUAL:

                if (theOpMode.gamepad1.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                    lift2.setPower(theOpMode.gamepad1.right_trigger);

                }
                else if (theOpMode.gamepad1.left_trigger > .1) {
                    lift.setPower(-theOpMode.gamepad1.left_trigger);
                    lift2.setPower(-theOpMode.gamepad1.left_trigger);

                } else {
                    lift.setPower(0);
                    lift2.setPower(0);
                }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case LOW:
                lift.setPower(0);
                lift2.setPower(0);
                if (theOpMode.gamepad1.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                    lift2.setPower(theOpMode.gamepad1.right_trigger);
                    teleState = TeleState.MANUAL;
                }
                break;
            case UP:
                target = 500;
                controller.setPID(p, i, d);
                int curPos = lift.getCurrentPosition();
                double pid = controller.calculate(curPos, target);
                double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
                double power = pid + ff;
                lift.setPower(power);
                lift2.setPower(power);
                if (lift.getCurrentPosition()-target <= 30) {
                    lift.setPower(0);
                    lift2.setPower(0);
                    target = lift.getCurrentPosition();
                    teleState = TeleState.START;
                }
            break;
            case SPECIMEN:


            default: teleState = TeleState.START;
        }

        theOpMode.telemetry.addData("LiftState", liftState);
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
        theOpMode.telemetry.update();


    }


    public void soloControls() {
        switch (teleState) {
            case START:
                if (theOpMode.gamepad1.right_trigger > 0.1 || theOpMode.gamepad1.left_trigger > 0.1 || theOpMode.gamepad2.right_trigger > 0.1 || theOpMode.gamepad2.left_trigger > 0.1) {
                    teleState = TeleState.MANUAL;
                }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case MANUAL:

            if (theOpMode.gamepad1.right_trigger > .1) {
                lift.setPower(theOpMode.gamepad1.right_trigger);
                lift2.setPower(theOpMode.gamepad1.right_trigger);

            }
                else if (theOpMode.gamepad2.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad2.right_trigger);
                    lift2.setPower(theOpMode.gamepad2.right_trigger);

                }
            else if (theOpMode.gamepad1.left_trigger > .1) {
                lift.setPower(-theOpMode.gamepad1.left_trigger);
                lift2.setPower(-theOpMode.gamepad1.left_trigger);

            }
                else if (theOpMode.gamepad2.left_trigger > .1) {
                    lift.setPower(-theOpMode.gamepad2.left_trigger);
                    lift2.setPower(-theOpMode.gamepad2.left_trigger);

                }
                else {
                lift.setPower(0);
                lift2.setPower(0);
            }
                if (!liftTouch.getState()) {
                    teleState = TeleState.LOW;
                }
                break;
            case LOW:
                lift.setPower(0);
                lift2.setPower(0);
                if (theOpMode.gamepad1.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad1.right_trigger);
                    lift2.setPower(theOpMode.gamepad1.right_trigger);
                    teleState = TeleState.MANUAL;
                }
                if (theOpMode.gamepad2.right_trigger > .1) {
                    lift.setPower(theOpMode.gamepad2.right_trigger);
                    lift2.setPower(theOpMode.gamepad2.right_trigger);
                    teleState = TeleState.MANUAL;
                }
                break;
            default: teleState = TeleState.START;
        }

    }

    public class SpecDeliver implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    claw.setPosition(closed);
                    deliveryS.setPosition(backMid);
                    target = 310;
                    if (Math.abs(lift.getCurrentPosition() - target) < 35) {
                        claw.setPosition(open);
                        lift.setPower(0);
                        lift2.setPower(0);
                        liftState = LiftState.START;
                        return false;
                    }
                    break;
                case LIFTED:
                    if (deliveryTimer.seconds() >= .1) {
                        claw.setPosition(open);
                        lift.setPower(0);
                        lift2.setPower(0);
                        liftState = LiftState.START;
                        return false;
                    }
                    break;
                default:
                    liftState = LiftState.START;

            }
            lController.setPID(lp, li, ld);
            int curPos = lift.getCurrentPosition();
            double pid = lController.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch));
            double power = pid + ff;
            lift.setPower(power);
            lift2.setPower(power);
            theOpMode.telemetry.addData("LiftState", liftState);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.addData("power", power);
            theOpMode.telemetry.update();
            return true;
        }
    }

    public Action specDeliver() {
        return new SpecDeliver();
    }

    public class LiftUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    deliveryS.setPosition(midPos);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 980;
                    if (Math.abs(lift.getCurrentPosition() - target) < 55) {
                        deliveryS.setPosition(backSpec);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                    if (liftTimer.seconds() >= .05) {
                        claw.setPosition(open);
                    }
                    if (liftTimer.seconds() >= .07) {
                        deliveryS.setPosition(midPos);
                    }
                        if (liftTimer.seconds() >= .08) {
                            liftState = LiftState.START;
                            return false;

                    }
                        break;
                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power);
            lift2.setPower(power);
            theOpMode.telemetry.addData("LiftState", liftState);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.update();

            return true;
        }
    }

    public Action liftUp() {
        return new LiftUp();
    }

    public class LiftDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    //deliveryS.setPosition(midPos);
                    liftState = LiftState.DOWN;
                    liftTimer.reset();
                    break;
                case DOWN:
                    target = -40;
                    if (liftTimer.seconds() >= 2.5) {
                        return false;
                    }
                    if (Math.abs(lift.getCurrentPosition() - target) < 45 && !liftTouch.getState()) {
                        lift.setPower(-0.3);
                        lift2.setPower(-0.3);
                        liftState = LiftState.START;
                        return false;
                    }
                    break;

                    default:
                        liftState = LiftState.START;
            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power * .8);
            lift2.setPower(power * .8);
            return true;
        }
    }
    public Action liftDown() {
        return new LiftDown();
    }
    public class LiftMid implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 610;
                    deliveryS.setPosition(backMid);
                    if (Math.abs(lift.getCurrentPosition() - target) < 20) {
                        lift.setPower(0);
                        lift2.setPower(0);
                            liftState = LiftState.START;
                            return false;
                        }
                    break;
                default:
                    liftState = LiftState.START;

            }
            lController.setPID(lp, li, ld);
            int curPos = lift.getCurrentPosition();
            double pid = lController.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power * .7);
            lift2.setPower(power * .7);
            return true;
        }
    }
    public Action liftMid() {
        return new LiftMid();
    }
    public class Pickup implements Action {
        @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                    switch (liftState) {
                        case START:
                            liftState = LiftState.DOWN;
                            deliveryTimer.reset();
                            break;
                        case DOWN:
                            deliveryS.setPosition(specPos);
                            if (deliveryTimer.seconds() >= .43) {
                                claw.setPosition(open);
                                liftState = LiftState.START;
                                return false;
                            }
                            break;

                        default:
                            liftState = LiftState.START;
                    }
                    return true;
                }
        }
        public Action pickup() {
        return new Pickup();
        }
    public class LiftTest implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    deliveryS.setPosition(midPos);
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 600;
                    if (Math.abs(lift.getCurrentPosition() - target) < 25) {
                        deliveryS.setPosition(backSpec);
                        liftTimer.reset();
                        liftState = LiftState.LIFTED;
                    }
                    break;
                case LIFTED:
                        return false;
                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power);
            lift2.setPower(power);
            theOpMode.telemetry.addData("LiftState", liftState);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.update();

            return true;
        }
    }

    public Action liftTest() {
        return new LiftTest();
    }









    public class LiftPickup implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    claw.setPosition(closed);
                    deliveryTimer.reset();
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:

                        target = 150;
                        deliveryS.setPosition(backSpec);
                        if (Math.abs(lift.getCurrentPosition() - target) < 70) {
                            lift.setPower(0);
                            lift2.setPower(0);
                            liftState = LiftState.START;
                            return false;
                    }
                        break;

                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power * 1.6);
            lift2.setPower(power * 1.6);
            return true;
        }
    }
    public Action liftPickup() {
        return new LiftPickup();
    }






    public class SpecDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    liftTimer.reset();
                    //deliveryS.setPosition(midPos);
                    liftState = LiftState.DOWN;
                    break;
                case DOWN:
                    if (liftTimer.seconds() >= 1) {
                        return false;
                    }
                    target = 0;
                    if (Math.abs(lift.getCurrentPosition() - target) < 30) {
                        deliveryS.setPosition(backSpec);
                        claw.setPosition(open);
                        lift.setPower(-0.12);
                        lift2.setPower(-0.12);
                        liftState = LiftState.START;
                        return false;
                    }
                    break;

                default:
                    liftState = LiftState.START;
            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
            lift.setPower(power * .6);
            lift2.setPower(power * .6);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("current position", curPos);
            theOpMode.telemetry.update();
            return true;
        }
    }
    public Action specDown() {
        return new SpecDown();
    }






    public class EncoderTest implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (liftState) {
                case START:
                    liftState = LiftState.LIFT;
                    break;
                case LIFT:
                    target = 600;
                    break;
                case LIFTED:
                    return false;
                default:
                    liftState = LiftState.START;

            }
            controller.setPID(p, i, d);
            int curPos = lift.getCurrentPosition();
            double pid = controller.calculate(curPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerInch)) * f;
            double power = pid + ff;
//            lift.setPower(power);
//            lift2.setPower(power);
            theOpMode.telemetry.addData("LiftState", liftState);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.addData("power", power);
            theOpMode.telemetry.update();

            return true;
        }
    }

    public Action encoderTest() {
        return new EncoderTest();
    }
}



