package org.firstinspires.ftc.teamcode.NewRobot;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OldStuff.PIDF;

public class AutoOptimize {


    public enum ExtendState {
        START,
        RETRACT,
        EXTEND,
        EXTENDED,
        TRANSFER,
        LIFT,
        LIFTED,
        REJECT

    }

    ExtendState extendState = ExtendState.START;
    PIDController controller;
    public static double p = 0.005, i = 0.000001, d = 0.000008;
    public static int target;


    OpMode theOpMode;
    DcMotorEx extend;
    DcMotorEx collection;
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;
    public Servo door;
    Servo sweep;

    int extended = 565;
    int retracted = 5;
    int mid = 300;
    int shortPos = 100;
    double collect = .45;
    double transfer = .33;
    double xHeight = .29;


    double closed = .623;
    double open = .42;
    double backSpec = .71;
    double midMid = .4;
    double transferPos = 0;
    double midPos = .53;
    double lowerMid = .15;


    double in = .4;
    double out = .705;

    DigitalChannel cBeam;
    DigitalChannel dBeam;
    DigitalChannel lSwitch;
    DigitalChannel touch;
    ColorSensor colorSensor;
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime beamTimer = new ElapsedTime();
    ElapsedTime failTimer = new ElapsedTime();

    PIDController liftController;
    public static double lp = 0.01, li = 0, ld = 0.0001;
    public static double f = 0.01;
    public static int liftTarget;
    private final double ticksPerInch = (145.1) / (1.15 * 3.14);


    DcMotorEx lift;
    DcMotorEx lift2;

    public DigitalChannel liftTouch;
    ElapsedTime liftTimer = new ElapsedTime();

    public AutoOptimize(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        controller = new PIDController(p, i, d);
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        extend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extend.setDirection(DcMotorSimple.Direction.FORWARD);
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        rCollection.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");
        door = hardwareMap.get(Servo.class, "door");
        sweep = hardwareMap.get(Servo.class, "sweep");
        collection = hardwareMap.get(DcMotorEx.class, "collection");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");
        cBeam.setMode(DigitalChannel.Mode.INPUT);
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        lSwitch = hardwareMap.get(DigitalChannel.class, "lSwitch");
        lSwitch.setMode(DigitalChannel.Mode.INPUT);


        liftController = new PIDController(lp, li, ld);
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift2 = hardwareMap.get(DcMotorEx.class, "lift2");

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftTouch = hardwareMap.get(DigitalChannel.class, "lTouch");
        liftTouch.setMode(DigitalChannel.Mode.INPUT);


        //Drivetrain turning test
        leftFront = hardwareMap.get(DcMotorEx.class, "leftDrive");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBackDrive");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBackDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightDrive");


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


    }


    public class SpeedCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.35);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    door.setPosition(.15);
                    target = extended;

                    if (Math.abs(extend.getCurrentPosition() - target) < 450) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case EXTENDED:
                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    target = extended;
                    if (beamTimer.seconds() > 0.8 && cBeam.getState()) {
                        target = mid;
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (beamTimer.seconds() > 1.5 && cBeam.getState()) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            collection.setPower(.95);
                        }
                        if (failTimer.seconds() >= 2.5) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }

                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case RETRACT:
                    target = retracted;
                    claw.setPosition(open);

                    if (beamTimer.seconds() >= .16 && (Math.abs(extend.getCurrentPosition() - retracted) < 580)) {
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        return false;
                    }
                    break;
                case REJECT:
                    target = 520;
                    collection.setPower(.3);
                    if (beamTimer.seconds() >= .2 && cBeam.getState()) {
                        collection.setPower(.7);
                        extendState = ExtendState.EXTEND;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);
            theOpMode.telemetry.update();
            return true;
        }

    }

    public Action speedCollect() {
        return new SpeedCollect();
    }

    public class SpeedCollect2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    liftTarget = -40;
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    collection.setPower(0);
                    extendState = ExtendState.RETRACT;
                    transferTimer.reset();

                    break;
                // Rotate to collecting position
                case RETRACT:
                    liftTarget = -40;
                    target = -15;
                    claw.setPosition(open);

                    if (Math.abs(extend.getCurrentPosition() - retracted) < 100) {
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 70) {
                            door.setPosition(.4);
                        }
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 50 && lift.getCurrentPosition() < 60) {
                            collection.setPower(.5);

                        }


                        if (!dBeam.getState()) {
                            collection.setPower(-.9);
                            door.setPosition(.15);
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.TRANSFER;
                        }
                        if (transferTimer.seconds() >= 0.7 && dBeam.getState()) {
                            extendState = ExtendState.TRANSFER;
                        }
                    }


                    break;
                case TRANSFER:
                    collection.setPower(-.8);
                    target = shortPos;
                    deliveryS.setPosition(midPos);
                    claw.setPosition(closed);
                    extendState = ExtendState.LIFT;
                    break;
                case LIFT:
                    liftTarget = 980;
                    if (Math.abs(lift.getCurrentPosition() - liftTarget) < 500) {
                        collection.setPower(0);
                    }
                    if (Math.abs(lift.getCurrentPosition() - liftTarget) < 65) {
                        deliveryS.setPosition(backSpec);
                        liftTimer.reset();
                        extendState = ExtendState.LIFTED;
                    }
                    break;
                case LIFTED:
                    if (liftTimer.seconds() >= .05) {
                        claw.setPosition(open);
                    }
                    if (liftTimer.seconds() >= .08) {
                        deliveryS.setPosition(midPos);
                        liftTarget = -30;
                    }
                    if (liftTimer.seconds() >= .09) {
                        liftTarget = -30;
                        extendState = ExtendState.START;
                        return false;

                    }
                    break;
                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);


            liftController.setPID(lp, li, ld);
            int lCurPos = lift.getCurrentPosition();
            double pid = liftController.calculate(lCurPos, liftTarget);
            double ff = Math.cos(Math.toRadians(liftTarget / ticksPerInch)) * f;
            double lPower = pid + ff;
            lift.setPower(lPower);
            lift2.setPower(lPower);
            theOpMode.telemetry.addData("target", liftTarget);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.update();
            return true;

        }

    }

    public Action speedCollect2() {
        return new SpeedCollect2();
    }


    public class SpeedCollect1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.35);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);
                    sweep.setPosition(out);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    door.setPosition(.15);
                    target = extended;

                    if (Math.abs(extend.getCurrentPosition() - target) < 500) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case EXTENDED:
                    target = extended;
                    if (beamTimer.seconds() > 1) {
                        target = mid;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        collection.setPower(-.9);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            if (beamTimer.seconds() >= 1.2) {
                                target = extended;
                                beamTimer.reset();
                                collection.setPower(.95);
                            }
                        }
                        if (failTimer.seconds() >= 3.1) {
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                    }

                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case RETRACT:
                    claw.setPosition(open);
                    if (cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    if (beamTimer.seconds() >= .1 && !((double) colorSensor.blue() / colorSensor.red() >= 1.1)) {
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        return false;
                    }
                    break;
                case REJECT:
                    target = 520;
                    collection.setPower(.3);
                    if (beamTimer.seconds() >= .2 && cBeam.getState()) {
                        collection.setPower(.7);
                        extendState = ExtendState.EXTEND;
                    }
                    if (beamTimer.seconds() >= 1 && !cBeam.getState()) {
                        collection.setPower(-.7);
                        extendState = ExtendState.EXTEND;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);
            theOpMode.telemetry.update();
            return true;
        }

    }

    public Action speedCollect1() {
        return new SpeedCollect1();
    }


    public class LiftCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                case START:
                    claw.setPosition(closed);
                    deliveryS.setPosition(midPos);
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    extendState = ExtendState.LIFT;
                    break;

                case LIFT:
                    liftTarget = 975;
                    target = 320;
                    if (Math.abs(lift.getCurrentPosition() - liftTarget) < 55) {
                        deliveryS.setPosition(backSpec);
                        liftTimer.reset();
                        extendState = ExtendState.LIFTED;
                    }
                    if (Math.abs(extend.getCurrentPosition() - target) < 35) {
                        extend.setPower(0);
                        beamTimer.reset();
                        failTimer.reset();
                    }
                    break;
                case LIFTED:
                    door.setPosition(.15);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    target = extended;

                    collection.setPower(.95);


                    if (!cBeam.getState()) {
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();


                    }
                    if (beamTimer.seconds() > 0.85) {
                        target = shortPos;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            target = extended;
                            beamTimer.reset();
                        }
                        if (failTimer.seconds() >= 1.6) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                    }
                    if (liftTimer.seconds() >= .05) {
                        claw.setPosition(open);
                    }
                    if (liftTimer.seconds() >= .07) {
                        deliveryS.setPosition(midMid);
                        liftTarget = -40;
                    }


                    break;
                case RETRACT:
                    liftTarget = -45;
                    claw.setPosition(open);

                    if (Math.abs(extend.getCurrentPosition() - retracted) < 120) {
                        door.setPosition(.3);
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 70) {
                            door.setPosition(.4);
                        }
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 40 && !liftTouch.getState()) {
                            collection.setPower(.55);
                        }
                        if (!dBeam.getState()) {
                            target = 80;
                            door.setPosition(.15);
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            collection.setPower(-.9);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.START;
                            return false;
                        }
                    }

                    break;

            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);


            liftController.setPID(lp, li, ld);
            int lCurPos = lift.getCurrentPosition();
            double pid = liftController.calculate(lCurPos, liftTarget);
            double ff = Math.cos(Math.toRadians(liftTarget / ticksPerInch)) * f;
            double lPower = pid + ff;
            lift.setPower(lPower);
            lift2.setPower(lPower);
            theOpMode.telemetry.addData("target", liftTarget);
            theOpMode.telemetry.addData("Current Position", lift.getCurrentPosition());
            theOpMode.telemetry.update();
            return true;
        }
    }

    public Action liftCollect() {
        return new LiftCollect();
    }


    public class SpeedCollect3 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    door.setPosition(.35);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);
                    sweep.setPosition(out);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    door.setPosition(.15);
                    target = extended;

                    if (Math.abs(extend.getCurrentPosition() - target) < 350) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case EXTENDED:
                    target = extended;

                    if (beamTimer.seconds() > 1) {
                        target = mid;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        collection.setPower(-.9);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            if (beamTimer.seconds() >= 1.2) {
                                target = extended;
                                beamTimer.reset();
                                collection.setPower(.95);
                            }
                        }
                    }


                    if (failTimer.seconds() >= 2) {
                        leftFront.setPower(-.6);
                        leftBack.setPower(-.6);
                        rightFront.setPower(.6);
                        rightBack.setPower(.6);
                    }
                    if (failTimer.seconds() >= 2.25) {
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                    if (failTimer.seconds() >= 4.2) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        deliveryS.setPosition(midPos);
                        return false;
                    }


                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case RETRACT:
                    claw.setPosition(open);
                    if (cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    if (beamTimer.seconds() >= .1 && !((double) colorSensor.blue() / colorSensor.red() >= 1.1)) {
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        return false;
                    }
                    break;
                case REJECT:
                    target = 520;
                    collection.setPower(.4);
                    if (beamTimer.seconds() >= .2 && cBeam.getState()) {
                        collection.setPower(.7);
                        extendState = ExtendState.EXTEND;
                    }
                    if (beamTimer.seconds() >= 1 && !cBeam.getState()) {
                        collection.setPower(-.7);
                        extendState = ExtendState.EXTEND;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p,i,d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State",extendState);
            theOpMode.telemetry.addData("Beam timer",beamTimer);
            theOpMode.telemetry.addData("rightFront",rightFront.getCurrentPosition());
            theOpMode.telemetry.addData("failTimer",failTimer);
            theOpMode.telemetry.update();
            return true;
        }
    }



    public Action speedCollect3() {
        return new SpeedCollect3();
    }


    public class SpeedCollect4 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.35);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);
                    sweep.setPosition(out);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    door.setPosition(.15);
                    target = extended;

                    if (Math.abs(extend.getCurrentPosition() - target) < 350) {
                        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case EXTENDED:
                    target = extended;

                    if (beamTimer.seconds() > 1) {
                        target = mid;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        collection.setPower(-.9);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            if (beamTimer.seconds() >= 1.2) {
                                target = extended;
                                beamTimer.reset();
                                collection.setPower(.95);
                            }
                        }
                    }


                    if (failTimer.seconds() >= 2) {
                        leftFront.setPower(-.6);
                        leftBack.setPower(-.6);
                        rightFront.setPower(.6);
                        rightBack.setPower(.6);
                    }
                    if (rightFront.getCurrentPosition() < -100) {
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                    if (failTimer.seconds() >= 3.1) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        deliveryS.setPosition(midPos);
                        return false;
                    }


            if (!cBeam.getState()) {
                door.setPosition(.15);
                deliveryS.setPosition(lowerMid);
                target = retracted;
                rCollection.setPosition(xHeight);
                lCollection.setPosition(xHeight);
                //target = retracted;
                collection.setPower(0);
                extendState = ExtendState.RETRACT;
                beamTimer.reset();
            }
            if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                door.setPosition(.45);
                extendState = ExtendState.REJECT;
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
            break;
            case RETRACT:
            claw.setPosition(open);
            if (cBeam.getState()) {
                extendState = ExtendState.EXTEND;
            }
            if ((double) colorSensor.blue() / colorSensor.red() >= 1.1) {
                door.setPosition(.45);
                extendState = ExtendState.REJECT;
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
            if (beamTimer.seconds() >= .1 && !((double) colorSensor.blue() / colorSensor.red() >= 1.1)) {
                collection.setPower(0);
                target = retracted;
                extendState = ExtendState.START;
                return false;
            }
            break;
            case REJECT:
            target = 520;
            collection.setPower(.4);
            if (beamTimer.seconds() >= .2 && cBeam.getState()) {
                collection.setPower(.7);
                extendState = ExtendState.EXTEND;
            }
            if (beamTimer.seconds() >= 1 && !cBeam.getState()) {
                collection.setPower(-.7);
                extendState = ExtendState.EXTEND;
            }
            break;

            default:
            extendState = ExtendState.START;
        }
        controller.setPID(p,i,d);
        int curPos = extend.getCurrentPosition();
        double power = controller.calculate(curPos, target);
        extend.setPower(power);
        theOpMode.telemetry.addData("Current State",extendState);
        theOpMode.telemetry.addData("Beam timer",beamTimer);
        theOpMode.telemetry.addData("rightFront",rightFront.getCurrentPosition());
        theOpMode.telemetry.addData("failTimer",failTimer);
        theOpMode.telemetry.update();
        return true;
    }
}



public Action speedCollect4() {
    return new SpeedCollect4();
}
















    public class SpeedCollectBlue implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    door.setPosition(.35);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);
                    sweep.setPosition(out);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    door.setPosition(.15);
                    target = extended;

                    if (Math.abs(extend.getCurrentPosition() - target) < 350) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if (((double) colorSensor.red() / colorSensor.blue()) >= 1.4 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.2)&& !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case EXTENDED:
                    target = extended;

                    if (beamTimer.seconds() > 1) {
                        target = mid;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        collection.setPower(-.9);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            if (beamTimer.seconds() >= 1.2) {
                                target = extended;
                                beamTimer.reset();
                                collection.setPower(.95);
                            }
                        }
                    }


                    if (failTimer.seconds() >= 2) {
                        leftFront.setPower(-.6);
                        leftBack.setPower(-.6);
                        rightFront.setPower(.6);
                        rightBack.setPower(.6);
                    }
                    if (failTimer.seconds() >= 2.25) {
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                    if (failTimer.seconds() >= 4.2) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        deliveryS.setPosition(midPos);
                        return false;
                    }


                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if (((double) colorSensor.red() / colorSensor.blue()) >= 1.4 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.2) && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }
                    break;
                case RETRACT:
                    claw.setPosition(open);
                    if (cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if (((double) colorSensor.red() / colorSensor.blue()) >= 1.4 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.2)) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                    if (beamTimer.seconds() >= .1) {
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        return false;
                    }
                    break;
                case REJECT:
                    target = 520;
                    collection.setPower(.4);
                    if (beamTimer.seconds() >= .2 && cBeam.getState()) {
                        collection.setPower(.7);
                        extendState = ExtendState.EXTEND;
                    }
                    if (beamTimer.seconds() >= 1 && !cBeam.getState()) {
                        collection.setPower(-.7);
                        extendState = ExtendState.EXTEND;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p,i,d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State",extendState);
            theOpMode.telemetry.addData("Beam timer",beamTimer);
            theOpMode.telemetry.addData("rightFront",rightFront.getCurrentPosition());
            theOpMode.telemetry.addData("failTimer",failTimer);
            theOpMode.telemetry.update();
            return true;
        }
    }



    public Action speedCollectBlue() {
        return new SpeedCollectBlue();
    }






    public class UpACreek implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.35);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);
                    sweep.setPosition(out);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    door.setPosition(.15);
                    target = 550;

                    if (Math.abs(extend.getCurrentPosition() - target) < 250) {
                        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                    }
                    break;
                case EXTENDED:
                    target = 550;

                    if (beamTimer.seconds() > 1) {
                        target = mid;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        collection.setPower(-.9);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            if (beamTimer.seconds() >= 1.2) {
                                target = 550;
                                beamTimer.reset();
                                collection.setPower(.95);
                            }
                        }
                    }


                    if (failTimer.seconds() >= 2) {
                        leftFront.setPower(-.6);
                        leftBack.setPower(-.6);
                        rightFront.setPower(.6);
                        rightBack.setPower(.6);
                    }
                    if (rightFront.getCurrentPosition() < -100) {
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                    if (failTimer.seconds() >= 3.1) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        deliveryS.setPosition(midPos);
                        return false;
                    }


                    if (!cBeam.getState()) {
                        door.setPosition(.15);
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1 && !cBeam.getState()) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }
                    break;
                case RETRACT:
                    claw.setPosition(open);
                    if (cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.1) {
                        door.setPosition(.45);
                        extendState = ExtendState.REJECT;
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }
                    if (beamTimer.seconds() >= .1 && !((double) colorSensor.blue() / colorSensor.red() >= 1.1)) {
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        return false;
                    }
                    break;
                case REJECT:
                    target = 520;
                    collection.setPower(.4);
                    if (beamTimer.seconds() >= .2 && cBeam.getState()) {
                        collection.setPower(.7);
                        extendState = ExtendState.EXTEND;
                    }
                    if (beamTimer.seconds() >= 1 && !cBeam.getState()) {
                        collection.setPower(-.7);
                        extendState = ExtendState.EXTEND;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p,i,d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State",extendState);
            theOpMode.telemetry.addData("Beam timer",beamTimer);
            theOpMode.telemetry.addData("rightFront",rightFront.getCurrentPosition());
            theOpMode.telemetry.addData("failTimer",failTimer);
            theOpMode.telemetry.update();
            return true;
        }
    }



    public Action upACreek() {
        return new UpACreek();
    }


}

