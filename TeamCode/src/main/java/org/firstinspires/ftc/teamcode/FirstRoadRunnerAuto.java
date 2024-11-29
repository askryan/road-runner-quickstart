package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

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
                }
                else {
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

        public IClaw(HardwareMap hardwareMap) {
            iClaw = hardwareMap.get(Servo.class, "iClaw");
            iClaw.scaleRange(0, 1);
        }

    }

    public class IRotation {
        public Servo iRotation;

        public IRotation(HardwareMap hardwareMap) {
            iRotation = hardwareMap.get(Servo.class, "iRotation");
            iRotation.scaleRange(0, 1);
        }
    }

    public class IArm {
        public Servo iArmL, iArmR;

        public IArm(HardwareMap hardwareMap) {
            iArmL = hardwareMap.get(Servo.class, "L1");
            iArmR = hardwareMap.get(Servo.class, "R1");
            iArmL.scaleRange(0, 1);
            iArmR.scaleRange(0, 1);
        }

    }

    public class OArm {
        public Servo oArm;

        public OArm(HardwareMap hardwareMap) {
            oArm = hardwareMap.get(Servo.class, "oArm");
            oArm.scaleRange(0, 1);
        }

    }

    public class OClaw {
        public Servo oClaw;

        public OClaw(HardwareMap hardwareMap) {
            oClaw = hardwareMap.get(Servo.class, "oClaw");
            oClaw.scaleRange(0, 1);
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
    }

}
