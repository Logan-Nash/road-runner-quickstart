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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.Arrays;
import java.util.List;

@Disabled
@TeleOp
public class ANewNewBot extends LinearOpMode {
    private DcMotorEx left_lift;
    private DcMotorEx right_lift;
    public DcMotorEx arm = null;
    public DcMotorEx chain = null;
    public Servo wrist;
    public CRServo intake = null;

    public boolean wristOpen = true;
    public boolean lastBump = false;

    public static final double WRIST_OPEN_POS = 0;
    public static final double WRIST_CLOSED_POS = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(CRServo.class, "intake");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        chain = hardwareMap.get(DcMotorEx.class, "chain");
        left_lift = hardwareMap.get(DcMotorEx.class, "left_lift");
        right_lift = hardwareMap.get(DcMotorEx.class, "right_lift");
        right_lift.setDirection(DcMotorSimple.Direction.REVERSE);
        //left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        chain.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        chain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist.scaleRange(-1, 1);
        wrist.setPosition(1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, Math.toRadians(-90)));
            waitForStart();
            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
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

                right_lift.setPower(gamepad2.right_stick_y);
                left_lift.setPower((gamepad2.right_stick_y));

                /*if (gamepad1.a) {
                    left_lift.setPower(1);
                } else left_lift.setPower(0);
                if(gamepad1.b) {
                    right_lift.setPower(1);
                }else {
                    right_lift.setPower(0);
                }*/

                // Extend arm with triggers
                if (gamepad1.right_trigger>0.1) {
                    arm.setPower(0.7);
                } else if (gamepad1.left_trigger>0.1) {
                    arm.setPower(-0.7);
                } else {
                    arm.setPower(0);
                }

                // Control intake servo with triggers
                if (gamepad2.right_trigger>0.1) {
                    intake.setPower(-1.0);
                } else if (gamepad2.left_trigger>0.1) {
                    intake.setPower(1.0);
                } else {
                    intake.setPower(0);
                }

                if (gamepad2.a && !lastBump) {
                    wristOpen = !wristOpen;
                    if (wristOpen) {
                        wrist.setPosition(WRIST_OPEN_POS);
                    } else {
                        wrist.setPosition(WRIST_CLOSED_POS);
                    }
                }


                if (gamepad2.right_bumper) {
                    chain.setPower(0.7);
                } else if (gamepad2.left_bumper) {
                    chain.setPower(-0.7);
                } else {
                    chain.setPower(0);
                }

                drive.updatePoseEstimate();
                // Send telemetry data to the driver station
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.addData("LiftPos", left_lift.getCurrentPosition());
                telemetry.addData("Encoder Count=",chain.getCurrentPosition());
                telemetry.update();
            }
        } else {
            throw new RuntimeException();
        }
    }
}
