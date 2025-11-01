
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Drive extends LinearOpMode{
    public DcMotor leftFrontMotor;
    public DcMotor leftBackMotor;
    public DcMotor rightFrontMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftShooter;
    public DcMotor rightShooter;
    public DcMotor secondStage;
    public DcMotor intake;
    public CRServo rightFirstStage;
    public CRServo leftFirstStage;

    private ElapsedTime     runtime = new ElapsedTime();
    private double shooterPower = 0.8;

    @Override
    public void runOpMode(){
        leftFrontMotor = hardwareMap.dcMotor.get("left_front");
        leftBackMotor = hardwareMap.dcMotor.get("left_back");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front");
        rightBackMotor = hardwareMap.dcMotor.get("right_back");
        leftShooter = hardwareMap.dcMotor.get("left_shooter");
        rightShooter = hardwareMap.dcMotor.get("right_shooter");
        secondStage = hardwareMap.dcMotor.get("second_stage");
        intake = hardwareMap.dcMotor.get("intake");

        leftFirstStage = hardwareMap.get(CRServo.class, "left_first_stage");
        rightFirstStage = hardwareMap.get(CRServo.class, "right_first_stage");

        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        secondStage.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFirstStage.setDirection(CRServo.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.a){
                intake.setPower(1);
            } else if(gamepad2.b){
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if(gamepad2.left_bumper){
                leftFirstStage.setPower(1);
            } else {
                leftFirstStage.setPower(0);
            }

            if(gamepad2.right_bumper){
                rightFirstStage.setPower(1);
            } else {
                rightFirstStage.setPower(0);
            }

            if(gamepad2.dpadUpWasPressed()){
                shooterPower += 0.05;
                if(shooterPower > 1){
                    shooterPower = 1;
                }
            }

            if(gamepad2.dpadDownWasPressed()){
                shooterPower -= 0.05;
                if(shooterPower < 0){
                    shooterPower = 0;
                }
            }

            if(gamepad1.a){
                rightShooter.setPower(shooterPower);
                leftShooter.setPower(shooterPower);
                secondStage.setPower(1);
            } else {
                rightShooter.setPower(0);
                leftShooter.setPower(0);
                secondStage.setPower(0);
            }

            double perSpeed = 0.65;
            if(gamepad1.left_bumper){
                perSpeed = 0.2;
            } else if(gamepad1.right_bumper){
                perSpeed = 1;
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x*1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y)+Math.abs(x)+Math.abs(rx),1);
            double frontLeftPower=perSpeed*(y+x+rx)/denominator;
            double backLeftPower=perSpeed*(y-x+rx)/denominator;
            double frontRightPower=perSpeed*(y-x-rx)/denominator;
            double backRightPower=perSpeed*(y+x-rx)/denominator;

            leftFrontMotor.setPower(frontLeftPower);
            leftBackMotor.setPower(backLeftPower);
            rightFrontMotor.setPower(frontRightPower);
            rightBackMotor.setPower(backRightPower);

            telemetry.addData("Shooter Speed", "%5.2f", shooterPower);
            telemetry.update();

        }
    }
}
