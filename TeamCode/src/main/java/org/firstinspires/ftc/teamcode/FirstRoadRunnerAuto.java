package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "GoodAuto", group = "Roadrunner")

public class FirstRoadRunnerAuto {

    public class ISlides {
        public DcMotor iSlideL, iSlideR;

        public ISlides(HardwareMap hardwareMap) {
            iSlideL = hardwareMap.get(DcMotor.class, "iSlideL");
            iSlideR = hardwareMap.get(DcMotor.class, "iSlideR");

            iSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            iSlideL.setDirection(DcMotorSimple.Direction.FORWARD);

            iSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            iSlideR.setDirection(DcMotorSimple.Direction.REVERSE);

            iSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            iSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            iSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            iSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class ExtendISlides implements Action {

            public boolean initialized = false;
            public static final int MAX_INTAKE_SLIDE_POS = 1100;
            public static final double INTAKE_SLIDE_POWER = 0.4;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!initialized) {
                    iSlideL.setPower(INTAKE_SLIDE_POWER);
                    iSlideR.setPower(INTAKE_SLIDE_POWER);
                    initialized = !initialized;
                }

                int posL = iSlideL.getCurrentPosition();
                int posR = iSlideR.getCurrentPosition();

                telemetryPacket.put("Left intake slide position", posL);
                telemetryPacket.put("Right intake slide position", posR);

                if (posL < MAX_INTAKE_SLIDE_POS && posR < MAX_INTAKE_SLIDE_POS) {
                    return true;
                } else {
                    iSlideL.setPower(0);
                    iSlideR.setPower(0);
                    return false;
                }
            }
        }

        public Action extendISlides() {
            return new ExtendISlides();
        }


    }

    public class IClaw {
        public Servo iClaw;
        public double minPos = 0; // needs manual testing II
        public double maxPos = 1; // needs manual testing II

        public IClaw(HardwareMap hardwareMap) {
            iClaw = hardwareMap.get(Servo.class, "iClaw");
            iClaw.scaleRange(minPos, maxPos);
        }

        public class OpenIClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                iClaw.setPosition(1);
                telemetryPacket.put("Intake claw position", maxPos);
                return false;
            }
        }

        public Action openIClaw() {
            return new OpenIClaw();
        }

        public class CloseIClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                iClaw.setPosition(0);
                telemetryPacket.put("Intake claw position", minPos);
                return false;
            }
        }

        public Action closeIClaw() {
            return new CloseIClaw();
        }
    }

    public class IRotation {
        public Servo iRotation;
        public double minPos = 0; // needs manual testing II
        public double maxPos = 1; // needs manual testing II

        public IRotation(HardwareMap hardwareMap) {
            iRotation = hardwareMap.get(Servo.class, "iRotation");
            iRotation.scaleRange(minPos, maxPos);
        }

    }

    public class IArm {
        public Servo iArmL, iArmR;

        public double minPosL = 0; // needs manual testing II
        public double maxPosL = 1; // needs manual testing II

        public double minPosR = 0; // needs manual testing II
        public double maxPosR = 1; // needs manual testing II

        public IArm(HardwareMap hardwareMap) {
            iArmL = hardwareMap.get(Servo.class, "L1");
            iArmR = hardwareMap.get(Servo.class, "R1");
            iArmL.scaleRange(minPosL, maxPosL);
            iArmR.scaleRange(minPosR, maxPosR);
        }

    }

    public class OArm {
        public Servo oArm;
        public double minPos = 0; // needs manual testing II
        public double maxPos = 1; // needs manual testing II

        public OArm(HardwareMap hardwareMap) {
            oArm = hardwareMap.get(Servo.class, "oArm");
            oArm.scaleRange(minPos, maxPos);
        }

        public class ExtendOArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                oArm.setPosition(1);
                telemetryPacket.put("Outtake claw position", maxPos);
                return false;
            }
        }

        public Action extendOArm() {
            return new ExtendOArm();
        }

        public class RetractOArm implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                oArm.setPosition(0);
                telemetryPacket.put("Outtake claw position", minPos);
                return false;
            }
        }

        public Action retractOArm() {
            return new RetractOArm();
        }

    }

    public class OClaw {
        public Servo oClaw;
        public double minPos = 0; // needs manual testing II
        public double maxPos = 1; // needs manual testing II

        public OClaw(HardwareMap hardwareMap) {
            oClaw = hardwareMap.get(Servo.class, "oClaw");
            oClaw.scaleRange(minPos, maxPos);
        }

        public class OpenOClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                oClaw.setPosition(1);
                telemetryPacket.put("Outtake claw position", maxPos);
                return false;
            }
        }

        public Action openOClaw() {
            return new OpenOClaw();
        }

        public class CloseOClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                oClaw.setPosition(0);
                telemetryPacket.put("Outtake claw position", minPos);
                return false;
            }
        }

        public Action closeOClaw() {
            return new CloseOClaw();
        }
    }

    public class OSlides {
        public DcMotor oSlideL, oSlideR;

        public OSlides(HardwareMap hardwareMap) {
            oSlideL = hardwareMap.get(DcMotor.class, "oSlideL");
            oSlideR = hardwareMap.get(DcMotor.class, "oSlideR");

            oSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            oSlideL.setDirection(DcMotorSimple.Direction.FORWARD);

            oSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            oSlideR.setDirection(DcMotorSimple.Direction.REVERSE);

            oSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            oSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            oSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            oSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class OSlidesBottom implements Action {

            public boolean initialized = false;
            public static final int BOTTOM_OUTTAKE_SLIDE_POS = 0;
            public static final double OUTTAKE_SLIDE_POWER = 0.5;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!initialized) {
                    oSlideL.setPower(-1 * OUTTAKE_SLIDE_POWER);
                    oSlideR.setPower(-1 * OUTTAKE_SLIDE_POWER);
                    initialized = !initialized;
                }

                int posL = oSlideL.getCurrentPosition();
                int posR = oSlideR.getCurrentPosition();

                telemetryPacket.put("Left outtake slide position", posL);
                telemetryPacket.put("Right outtake slide position", posR);

                if (posL > BOTTOM_OUTTAKE_SLIDE_POS && posR > BOTTOM_OUTTAKE_SLIDE_POS) {
                    return true;
                } else {
                    oSlideL.setPower(0);
                    oSlideR.setPower(0);
                    return false;
                }
            }
        }

        public Action oSlidesBottom() {
            return new OSlidesBottom();
        }

        public class ExtendOSlidesHBar implements Action {

            public boolean initialized = false;
            public static final int HBAR_OUTTAKE_SLIDE_POS = 1500;
            public static final double OUTTAKE_SLIDE_POWER = 0.5;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!initialized) {
                    oSlideL.setPower(OUTTAKE_SLIDE_POWER);
                    oSlideR.setPower(OUTTAKE_SLIDE_POWER);
                    initialized = !initialized;
                }

                int posL = oSlideL.getCurrentPosition();
                int posR = oSlideR.getCurrentPosition();

                telemetryPacket.put("Left outtake slide position", posL);
                telemetryPacket.put("Right outtake slide position", posR);

                if (posL < HBAR_OUTTAKE_SLIDE_POS && posR < HBAR_OUTTAKE_SLIDE_POS) {
                    return true;
                } else {
                    oSlideL.setPower(0);
                    oSlideR.setPower(0);
                    return false;
                }
            }
        }

        public Action extendOSlidesHBar() {
            return new ExtendOSlidesHBar();
        }

        public class LowerOSlidesToHook implements Action {

            public boolean initialized = false;
            public static final int HBAR_OUTTAKE_SLIDE_POS = 1400;
            public static final double OUTTAKE_SLIDE_POWER = 0.7;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!initialized) {
                    oSlideL.setPower(-1 * OUTTAKE_SLIDE_POWER);
                    oSlideR.setPower(-1 * OUTTAKE_SLIDE_POWER);
                    initialized = !initialized;
                }

                int posL = oSlideL.getCurrentPosition();
                int posR = oSlideR.getCurrentPosition();

                telemetryPacket.put("Left outtake slide position", posL);
                telemetryPacket.put("Right outtake slide position", posR);

                if (posL > HBAR_OUTTAKE_SLIDE_POS || posR > HBAR_OUTTAKE_SLIDE_POS) {
                    return true;
                } else {
                    oSlideL.setPower(0);
                    oSlideR.setPower(0);
                    return false;
                }
            }
        }

        public Action lowerOSlidesToHook() {
            return new LowerOSlidesToHook();
        }

        public class ExtendOSlidesPickup implements Action {

            public boolean initialized = false;
            public static final int HBAR_OUTTAKE_SLIDE_POS = 800;
            public static final double OUTTAKE_SLIDE_POWER = 0.5;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!initialized) {
                    oSlideL.setPower(OUTTAKE_SLIDE_POWER);
                    oSlideR.setPower(OUTTAKE_SLIDE_POWER);
                    initialized = !initialized;
                }

                int posL = oSlideL.getCurrentPosition();
                int posR = oSlideR.getCurrentPosition();

                telemetryPacket.put("Left outtake slide position", posL);
                telemetryPacket.put("Right outtake slide position", posR);

                if (posL < HBAR_OUTTAKE_SLIDE_POS && posR < HBAR_OUTTAKE_SLIDE_POS) {
                    return true;
                } else {
                    oSlideL.setPower(0);
                    oSlideR.setPower(0);
                    return false;
                }
            }
        }

        public Action extendOSlidesPickup() {
            return new ExtendOSlidesPickup();
        }

    }
}