package TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Disabled
@TeleOp
public abstract class NewBotTest extends LinearOpMode {
    private DcMotorEx right_lift;
    private DcMotorEx left_lift;
    public DcMotorEx arm = null;
    public DcMotorEx chain = null;
    public Servo wrist = null;
    public CRServo intake = null;
    public static final int ARM_POS_INIT = 300;
    public static final int ARM_POS_INTAKE = 450;
    public static final int ARM_POS_WALL_GRAB = 1100;
    public static final int ARM_POS_WALL_UNHOOK = 1700;
    public static final int ARM_POS_HOVER_HIGH = 2600;
    public static final int ARM_POS_CLIP_HIGH = 2100;
    public static final int ARM_POS_LOW_BASKET = 2500;

    public static final int WRIST_POS_INIT = 0;
    public static final int WRIST_POS_SAMPLE = 270;
    public static final int WRIST_POS_SPEC = 10;

    public static final double INTAKE_OPEN_POS = -1;
    public static final double INTAKE_CLOSED_POS = 1;}

    /*public enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
        }
        private RobotState currentState = RobotState.INIT;

        public boolean clawOpen = true;
        public boolean lastBump = false;
        public boolean lastHook = false;
        public boolean lastGrab = false;

        private int targetArm = 0;
        private int targetWrist = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        left_lift = hardwareMap.get(DcMotorEx.class, "left_lift");
        right_lift = hardwareMap.get(DcMotorEx.class, "right_lift");

        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        chain = hardwareMap.get(DcMotorEx.class, "chain");



        left_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(CRServo.ZeroPowerBehavior.BRAKE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, Math.toRadians(-90)));
            waitForStart();
            /* while (opModeIsActive()) {
                switch (currentState) {
                    case INIT:
                        targetArm = ARM_POS_INIT;
                        targetWrist = WRIST_POS_INIT;
                        telemetry.addData("State", "INIT");
                        break;
                    case INTAKE:
                        targetArm = ARM_POS_INTAKE;
                        targetWrist = WRIST_POS_SAMPLE;
                        telemetry.addData("State", "INTAKE");
                        break;
                    case WALL_GRAB:
                        targetArm = ARM_POS_WALL_GRAB;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "WALL_GRAB");
                        break;
                    case WALL_UNHOOK:
                        targetArm = ARM_POS_WALL_UNHOOK;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "WALL_UNHOOK");
                        break;

                    case HOVER_HIGH:
                        targetArm = ARM_POS_HOVER_HIGH;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "HOVER_HIGH");
                        break;

                    case CLIP_HIGH:
                        targetArm = ARM_POS_CLIP_HIGH;
                        targetWrist = WRIST_POS_SPEC;
                        telemetry.addData("State", "CLIP_HIGH");
                        break;
                    case LOW_BASKET:
                        targetArm = ARM_POS_LOW_BASKET;
                        targetWrist = WRIST_POS_SAMPLE;
                        telemetry.addData("State", "LOW_BASKET");
                        break;
                    case MANUAL:
                        telemetry.addData("State", "MANUAL");
                        break;
                }

                // Handle state transitions based on gamepad input
                if (gamepad2.a) {
                    currentState = NewBotTest.RobotState.INTAKE;
                } else if (gamepad2.b && !lastGrab) {
                    if(currentState == NewBotTest.RobotState.WALL_GRAB){
                        currentState = NewBotTest.RobotState.WALL_UNHOOK;
                    }else{
                        currentState = NewBotTest.RobotState.WALL_GRAB;
                    }
                } else if (gamepad2.y && !lastHook) {
                    if(currentState == NewBotTest.RobotState.HOVER_HIGH){
                        currentState = NewBotTest.RobotState.CLIP_HIGH;
                    }else{
                        currentState = NewBotTest.RobotState.HOVER_HIGH;
                    }
                } else if (gamepad2.x) {
                    currentState = NewBotTest.RobotState.LOW_BASKET;
                } else if (gamepad2.left_bumper) {
                    currentState = NewBotTest.RobotState.INIT;
                } else if (gamepad2.dpad_up){ //manual control
                    currentState = NewBotTest.RobotState.MANUAL;
                    targetArm += 10;
                } else if (gamepad2.dpad_down){
                    currentState = NewBotTest.RobotState.MANUAL;
                    targetArm -= 10;
                } else if (gamepad2.dpad_left){
                    currentState = NewBotTest.RobotState.MANUAL;
                    targetWrist += 1;
                } else if (gamepad2.dpad_right){
                    currentState = NewBotTest.RobotState.MANUAL;
                    targetWrist -= 1;
                } */
/*
                lastGrab = gamepad2.b;
                lastHook = gamepad2.y;


                // Handle state transitions based on gamepad input
               if (gamepad2.a) {
                   left_lift.setPower(1);
                   right_lift.setPower(1);
               } else if (gamepad2.b) {
                   left_lift.setPower(-1);
                   right_lift.setPower(-1);
               } else {
                   left_lift.setPower(0);
                   right_lift.setPower(0);
               }

                // Control intake servo with triggers
                /* if (gamepad2.right_trigger>0.1) {
                    intake.setPower(1.0);
                } else if (gamepad2.left_trigger>0.1) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0);
                } */



            // Servo power values for intake directions
            /* public class NewBotTest INTAKE_POWER = 1.0;
            public class NewBotTest REVERSE_POWER = -1.0;
            public class final double STOP_POWER = 0.0;


                // Get trigger values
                float rightTrigger = gamepad2.right_trigger;
                float leftTrigger = gamepad2.left_trigger;

                // Check if right trigger is pressed to intake
                if (rightTrigger > 0.1) {
                    intake.setPosition(INTAKE_POWER);
                }
                // Check if left trigger is pressed to reverse
                else if (leftTrigger > 0.1) {
                    intake.setPosition(REVERSE_POWER);
                }
                // Stop the servo if no trigger is pressed
                else {
                    intake.setPosition(STOP_POWER);
                }

                // Telemetry for debugging
                telemetry.addData("Right Trigger", rightTrigger);
                telemetry.addData("Left Trigger", leftTrigger);
                telemetry.update();
            }
        }


                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                gamepad1.left_stick_y,
                                gamepad1.left_stick_x
                        ),
                        -gamepad1.right_stick_x

                ));
                if (gamepad1.right_bumper) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y/4,
                                    -gamepad1.left_stick_x/4
                            ),
                            -gamepad1.right_stick_x/2
                    ));
                }



                drive.updatePoseEstimate();
                // Send telemetry data to the driver station
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();
            }
        } else {
            throw new RuntimeException();
        }
    }
}

             */
