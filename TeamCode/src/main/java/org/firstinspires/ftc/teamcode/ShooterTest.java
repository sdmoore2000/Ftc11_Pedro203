
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ShooterTest extends LinearOpMode{
    public DcMotor leftShooter;
    public DcMotor rightShooter;
    public DcMotor secondStage;
    public Servo leftLift;
    public Servo rightLift;

    private ElapsedTime     runtime = new ElapsedTime();
    private double shooterPower = 0.8;
    private double leftPos = 0.01;
    private double rightPos = 0.03;

    @Override
    public void runOpMode(){
        leftShooter = hardwareMap.dcMotor.get("left_shooter");
        rightShooter = hardwareMap.dcMotor.get("right_shooter");
        secondStage = hardwareMap.dcMotor.get("second_stage");
        leftLift = hardwareMap.get(Servo.class, "left_lift");
        rightLift = hardwareMap.get(Servo.class, "right_lift");

        secondStage.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(Servo.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

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

            if(gamepad2.dpadRightWasPressed()){
                leftPos += 0.01;
                rightPos += 0.01;
                if(leftPos > 1){
                    leftPos = 1;
                }
                if(rightPos > 1){
                    rightPos = 1;
                }
            }

            if(gamepad2.dpadLeftWasPressed()){
                leftPos -= 0.01;
                rightPos -= 0.01;
                if(leftPos < 0){
                    leftPos = 0;
                }
                if(rightPos < 0){
                    rightPos = 0;
                }
            }

            if(gamepad2.x){
                leftLift.setPosition(0.13);
                rightLift.setPosition(0.13);
            }

            if(gamepad2.y){
                leftLift.setPosition(leftPos);
                rightLift.setPosition(rightPos);
            }

            if(gamepad2.a){
                rightShooter.setPower(shooterPower);
                leftShooter.setPower(shooterPower);
            } else {
                rightShooter.setPower(0);
                leftShooter.setPower(0);
            }

            if(gamepad2.b){
                secondStage.setPower(1);
            } else {
                secondStage.setPower(0);
            }

            telemetry.addData("Shooter Speed", "%5.2f", shooterPower);
            telemetry.addData("Left Pos", "%5.2f", leftPos);
            telemetry.addData("Right Pos", "%5.2f", rightPos);
            telemetry.update();

        }
    }
}
