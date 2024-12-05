package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;






// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "GoodAuto", group = "Roadrunner")
public class FirstRoadRunnerAuto extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public class OArm {
        public Servo oArm;
        public double minPos = 0.18; // needs manual testing II
        public double maxPos = 0.95; // needs manual testing II

        public OArm(HardwareMap hardwareMap) {
            oArm = hardwareMap.get(Servo.class, "oArm");
            oArm.scaleRange(minPos, maxPos);
        }

        public class ExtendOArm implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    oArm.setPosition(1);
                    initialized = true;
                }
                telemetryPacket.put("Outtake claw position", maxPos);
                return timer.milliseconds() < 800;
            }
        }

        public Action extendOArm() {
            return new ExtendOArm();
        }

        public class RetractOArm implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    oArm.setPosition(0);
                    initialized = true;
                }
                telemetryPacket.put("Outtake claw position", minPos);
                return timer.milliseconds() < 800;
            }
        }

        public Action retractOArm() {
            return new RetractOArm();
        }

    }

    public class OClaw {
        public Servo oClaw;
        public double minPos = 0.41; // needs manual testing II
        public double maxPos = 0.55; // needs manual testing II

        public OClaw(HardwareMap hardwareMap) {
            oClaw = hardwareMap.get(Servo.class, "oClaw");
            oClaw.scaleRange(minPos, maxPos);
        }

        public class OpenOClaw implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    oClaw.setPosition(1);
                    initialized = true;
                }
                telemetryPacket.put("Outtake claw position", maxPos);
                return timer.milliseconds() < 400;
            }
        }

        public Action openOClaw() {
            return new OpenOClaw();
        }

        public class CloseOClaw implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    oClaw.setPosition(0);
                    initialized = true;
                }

                telemetryPacket.put("Outtake claw position", minPos);
                return timer.milliseconds() < 400;
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
            public static final int BOTTOM_OUTTAKE_SLIDE_POS = 0; // needs manual testing II
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
            public static final int HBAR_OUTTAKE_SLIDE_POS = 1500; // needs manual testing II
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
            public static final int HBAR_OUTTAKE_SLIDE_POS = 1400; // needs manual testing II
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
            public static final int HBAR_OUTTAKE_SLIDE_POS = 800; // needs manual testing II
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

        public class LowerOSlidesPickup implements Action {

            public boolean initialized = false;
            public static final int HBAR_OUTTAKE_SLIDE_POS = 800; // needs manual testing II
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

                if (posL > HBAR_OUTTAKE_SLIDE_POS && posR > HBAR_OUTTAKE_SLIDE_POS) {
                    return true;
                } else {
                    oSlideL.setPower(0);
                    oSlideR.setPower(0);
                    return false;
                }
            }
        }

        public Action lowerOSlidesPickup() {
            return new LowerOSlidesPickup();
        }

    }


    @Override
    public void runOpMode() {
        double ninety = Math.toRadians(90);
        double oneEighty = Math.toRadians(180);
        double twoSeventy = Math.toRadians(270);

        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(20, -61, ninety); // needs manual testing II

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        OArm oArm = new OArm(hardwareMap);

        OClaw oClaw = new OClaw(hardwareMap);

        OSlides oSlides = new OSlides(hardwareMap);

        /*

        //https://learnroadrunner.com/trajectories.html#slowing-down-a-trajectory to adjust trajectory speed
        TrajectoryActionBuilder tab0 = drive.actionBuilder(initialPose)
                .splineToSplineHeading(new Pose2d(10, -34, twoSeventy), ninety) // position at the bar for the preload
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(36, -30, ninety), ninety) // intermediate stage in the path to first block
                .splineToConstantHeading(new Vector2d(48, -10), 0) // arrives north of the first block
                .setTangent(ninety)
                .lineToY(-56) // pushes the block
                .lineToY(-16) // goes up to try and reach the second block
                .splineToConstantHeading(new Vector2d(58, -14), Math.toRadians(300)) // goes north of the second block
                .setTangent(ninety)
                .lineToY(-56) // pushes the second block down
                .strafeToConstantHeading(new Vector2d(36, -61)) // goes to the pickup position for the first block
                .waitSeconds(0.2)
                .splineToSplineHeading(new Pose2d(10, -34, twoSeventy), ninety) // goes to the bar
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(36, -61), ninety) // goes to the pickup position for the second block
                .waitSeconds(0.2)
                .splineToSplineHeading(new Pose2d(10, -34, twoSeventy), ninety) // goes to the bar
                .waitSeconds(1)
                .strafeTo(new Vector2d(61, -61));// parks

         */

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose).
                splineToSplineHeading(new Pose2d(10, -34, twoSeventy), ninety);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(10, -34, twoSeventy))
                .splineToSplineHeading(new Pose2d(36, -30, ninety), ninety) // intermediate stage in the path to first block
                .splineToConstantHeading(new Vector2d(48, -10), 0) // arrives north of the first block
                .setTangent(ninety)
                .lineToY(-56) // pushes the block
                .lineToY(-16) // goes up to try and reach the second block
                .splineToConstantHeading(new Vector2d(58, -14), Math.toRadians(300)) // goes north of the second block
                .setTangent(ninety)
                .lineToY(-56); //pushes the second block down

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(58, -56, ninety))
                .strafeToConstantHeading(new Vector2d(36, -61)); // goes to the pickup position for the first block

        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(36, -61, ninety))
                .splineToSplineHeading(new Pose2d(10, -34, twoSeventy), ninety); // goes to hook the first picked up block

        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(10, -34, twoSeventy))
                .strafeToSplineHeading(new Vector2d(36, -61), ninety); // goes to the pickup position for the second block

        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(36, -61, ninety))
                .splineToSplineHeading(new Pose2d(10, -34, twoSeventy), ninety); // goes to the bar

        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(10, -34, twoSeventy))
                .strafeTo(new Vector2d(61, -61)); // parks

        //init actions
        Actions.runBlocking(new SequentialAction(oClaw.closeOClaw(), oArm.retractOArm()));

        waitForStart();

        if (isStopRequested()) return;

        Action act1 = tab1.build();
        Action act2 = tab2.build();
        Action act3 = tab3.build();
        Action act4 = tab4.build();
        Action act5 = tab5.build();
        Action act6 = tab6.build();
        Action park = tab7.build();

        ParallelAction preloadGoToBar = new ParallelAction(
                act1,
                oSlides.extendOSlidesHBar(),
                oArm.extendOArm()
        );

        SequentialAction goToBarAndHook = new SequentialAction(preloadGoToBar, oSlides.lowerOSlidesToHook(),
                oClaw.openOClaw(), oArm.retractOArm());

        ParallelAction lowerSlidesAndPush = new ParallelAction(oSlides.oSlidesBottom(), act2);

        ParallelAction firstPickup = new ParallelAction(act3, oArm.extendOArm(), oSlides.extendOSlidesPickup());

        ParallelAction firstHook = new ParallelAction(act4, oSlides.extendOSlidesHBar());

        SequentialAction actuallyHook = new SequentialAction(firstHook, oSlides.lowerOSlidesToHook(),
                oClaw.openOClaw(), oArm.retractOArm());

        ParallelAction goBackToPickup = new ParallelAction(act5, oSlides.lowerOSlidesPickup(), oArm.extendOArm());

        ParallelAction lastGoToHook = new ParallelAction(act6, oSlides.extendOSlidesHBar());

        SequentialAction hookSecondTime = new SequentialAction(lastGoToHook,
                oSlides.lowerOSlidesToHook(), oClaw.openOClaw(), oArm.retractOArm()); // Even though this is technically the third time, I'm counting the preload as the 0th time


        Actions.runBlocking(
                new SequentialAction(goToBarAndHook, lowerSlidesAndPush, firstPickup, oClaw.closeOClaw()
                , actuallyHook, goBackToPickup, oClaw.closeOClaw(), hookSecondTime, park)
        );


    }

}