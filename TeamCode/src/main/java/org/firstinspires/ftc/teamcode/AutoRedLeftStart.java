/*

Blue Goal: 20
Motif GPP: 21
Motif PGP: 22
Motif PPG: 23
Red Goal: 24

So basically, you lineup your robot in front of the motif april tag. It scans said April Tag and then gives you a value back. You then have three if/then statements where you pretty much
say "if the numeric value is 21, then run the GPP pathbuilder" and so on. Right now, though, the code just has movement. So whenever you get your shooting and intake mechanisms figured out, just add that code in the
designated function and call the function in whichever part of the pathbuilder it is needed. I hope this helps!
*/


package org.firstinspires.ftc.teamcode;

// FTC SDK

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "Auto Red Left Start", group = "Opmode")
@Configurable // Panels
@SuppressWarnings("FieldCanBeLocal") // Stop Android Studio from bugging about variables being predefined
public class AutoRedLeftStart extends LinearOpMode {
    // Add color sensor
    private NormalizedColorSensor leftColor;
    double hue;

    // Add items for shooting and intake
    public DcMotor leftShooter;
    public DcMotor rightShooter;
    public DcMotor secondStage;
    public DcMotor intake;
    public CRServo rightFirstStage;
    public CRServo leftFirstStage;

    // Initialize elapsed timer
    private final ElapsedTime runtime = new ElapsedTime();

    // Initialize poses
    private final Pose startPose = new Pose(80, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(80, 90, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 115 degree angle.
    private final Pose finalPose = new Pose(90, 56, Math.toRadians(90)); // Final Pose of our robot.
    private final Pose PPGPose = new Pose(100, 83.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose PPG2Pose = new Pose(104, 83.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark position 2.
    private final Pose PPG3Pose = new Pose(109, 83.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark position 3.
    private final Pose PPG4Pose = new Pose(115, 83.5, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark position 4.
    private final Pose PGPPose = new Pose(100, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose PGP2Pose = new Pose(104, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark position 2.
    private final Pose PGP3Pose = new Pose(109, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark position 3.
    private final Pose PGP4Pose = new Pose(115, 59.5, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark position 4.
    private final Pose GPPPose = new Pose(100, 35.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose GPP2Pose = new Pose(104, 35.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark position 2.
    private final Pose GPP3Pose = new Pose(109, 35.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark position 3.
    private final Pose GPP4Pose = new Pose(115, 35.5, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark position 4.
    // Initialize variables for paths

    private PathChain initialScorePPG;
    private PathChain grabPPG;
    private PathChain grab2PPG;
    private PathChain grab3PPG;
    private PathChain grab4PPG;
    private PathChain scorePPG;
    private PathChain initialScorePGP;
    private PathChain grabPGP;
    private PathChain grab2PGP;
    private PathChain grab3PGP;
    private PathChain grab4PGP;
    private PathChain scorePGP;
    private PathChain initialScoreGPP;
    private PathChain grabGPP;
    private PathChain grab2GPP;
    private PathChain grab3GPP;
    private PathChain grab4GPP;
    private PathChain scoreGPP;
    private PathChain finalPosition;


    //set April Tag values to specific patterns
    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag


    // Other variables
    private Pose currentPose; // Current pose of the robot
    private Follower follower; // Pedro Pathing follower
    private TelemetryManager panelsTelemetry; // Panels telemetry
    private int pathStatePPG; // Current state machine value
    private int pathStatePGP; // Current state machine value
    private int pathStateGPP; // Current state machine value

    private int foundID; // Current state machine value, dictates which one to run



    // Custom logging function to support telemetry and Panels
    private void log(String caption, Object... text) {
        if (text.length == 1) {
            telemetry.addData(caption, text[0]);
            panelsTelemetry.debug(caption + ": " + text[0]);
        } else if (text.length >= 2) {
            StringBuilder message = new StringBuilder();
            for (int i = 0; i < text.length; i++) {
                message.append(text[i]);
                if (i < text.length - 1) message.append(" ");
            }
            telemetry.addData(caption, message.toString());
            panelsTelemetry.debug(caption + ": " + message);
        }
    }

    // a place to put your intake and shooting functions
    public void initialIntakeArtifacts() {
        // Put your intake logic/functions here
        intake.setPower(1);
        leftFirstStage.setPower(1);
        rightFirstStage.setPower(1);
    }

    public void intakeArtifacts() {
        // Put your intake logic/functions here
        leftFirstStage.setPower(0);
        rightFirstStage.setPower(0);
        sleep(500);
    }

    public void stopIntakeArtifacts() {
        // Put your intake logic/functions here
        intake.setPower(0);
    }

    public void shootArtifactsLeft() {
        // Put your shooting logic/functions here
        leftShooter.setPower(0.7);
        rightShooter.setPower(0.7);
        sleep(1000);
        secondStage.setPower(1);
        sleep(2000);
        leftFirstStage.setPower(1);
        sleep(2000);
        leftFirstStage.setPower(0);
        rightFirstStage.setPower(1);
        sleep(2000);
        rightFirstStage.setPower(0);
        secondStage.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    public void shootArtifactsRight() {
        // Put your shooting logic/functions here
        leftShooter.setPower(0.7);
        rightShooter.setPower(0.7);
        sleep(1000);
        secondStage.setPower(1);
        sleep(2000);
        rightFirstStage.setPower(1);
        sleep(2000);
        rightFirstStage.setPower(0);
        leftFirstStage.setPower(1);
        sleep(2000);
        leftFirstStage.setPower(0);
        secondStage.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }


    @Override
    public void runOpMode() {
        // Initialize shooting and intake items
        leftShooter = hardwareMap.dcMotor.get("left_shooter");
        rightShooter = hardwareMap.dcMotor.get("right_shooter");
        secondStage = hardwareMap.dcMotor.get("second_stage");
        intake = hardwareMap.dcMotor.get("intake");

        leftFirstStage = hardwareMap.get(CRServo.class, "left_first_stage");
        rightFirstStage = hardwareMap.get(CRServo.class, "right_first_stage");

        leftColor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        secondStage.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFirstStage.setDirection(CRServo.Direction.REVERSE);
        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize Panels telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        initAprilTag();

        if (USE_WEBCAM) {
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        }

        // Log completed initialization to Panels and driver station (custom log function)
        log("Status", "Initialized");
        telemetry.update(); // Update driver station after logging

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        setpathStatePPG(0);
        setpathStatePGP(0);
        setpathStateGPP(0);
        runtime.reset();

        while (opModeIsActive()) {
            // Update Pedro Pathing and Panels every iteration
            follower.update();
            panelsTelemetry.update();
            currentPose = follower.getPose(); // Update the current pose
            targetFound = false;
            desiredTag = null;

            // Color sensing
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) leftColor).getLightDetected());
            NormalizedRGBA colors = leftColor.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());
            telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));

            //Using hue to detect color
            if(hue < 30){
                telemetry.addData("Color", "Red");
            }
            else if (hue < 60) {
                telemetry.addData("Color", "Orange");
            }
            else if (hue < 90){
                telemetry.addData("Color", "Yellow");
            }
            else if (hue < 150){
                telemetry.addData("Color", "Green");
            }
            else if (hue < 225){
                telemetry.addData("Color", "Blue");
            }
            else if (hue < 350){
                telemetry.addData("Color", "Purple");
            }
            else {
                telemetry.addData("Color", "Red");
            }

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == PPG_TAG_ID) {
                        // call lines for the PGP pattern
                        buildPathsPPG();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 21; // This should likely be PPG_TAG_ID or the corresponding state machine ID
                        break;  // don't look any further.
                    } else if (detection.id == PGP_TAG_ID) {
                        // call lines for the PGP pattern
                        buildPathsPGP();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 22; // This should likely be PGP_TAG_ID or the corresponding state machine ID
                        break;  // don't look any further.

                    } else if (detection.id == GPP_TAG_ID) {
                        // call lines for the GPP pattern
                        buildPathsGPP();
                        targetFound = true;
                        desiredTag = detection;
                        foundID = 23; // This should likely be GPP_TAG_ID or the corresponding state machine ID
                        break;  // don't look any further.
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    // Default to call lines for the PGP pattern
                    buildPathsPPG();
                    targetFound = true;
                    desiredTag = detection;
                    foundID = 21; // This should likely be PPG_TAG_ID or the corresponding state machine ID
                    break;  // don't look any further.
                }
            }


            // Update the state machine
            if (foundID == 21) { // Consider using the TAG_ID constants or a dedicated variable for which path was found
                updateStateMachinePPG();
            } else if (foundID == 22) {
                updateStateMachinePGP();
            } else if (foundID == 23) {
                updateStateMachineGPP();
            }


            // Log to Panels and driver station (custom log function)
            log("Elapsed", runtime.toString());
            log("X", currentPose.getX());
            log("Y", currentPose.getY());
            log("Heading", currentPose.getHeading());
            telemetry.update(); // Update the driver station after logging
        }
    }


    public void buildPathsPPG() {
        // basically just plotting the points for the lines that score the PPG pattern

        initialScorePPG = follower.pathBuilder() // Move to the scoring pose from the stating pose
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPPG = follower.pathBuilder() // Move to the grabbing pose from the scoring pose
                .addPath(new BezierLine(scorePose, PPGPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PPGPose.getHeading())
                .build();

        grab2PPG = follower.pathBuilder() // Move to the second grabbing pose from the first grabbing pose
                .addPath(new BezierLine(PPGPose, PPG2Pose))
                .setLinearHeadingInterpolation(PPGPose.getHeading(), PPG2Pose.getHeading())
                .build();

        grab3PPG = follower.pathBuilder() // Move to the third grabbing pose from the second grabbing pose
                .addPath(new BezierLine(PPG2Pose, PPG3Pose))
                .setLinearHeadingInterpolation(PPG2Pose.getHeading(), PPG3Pose.getHeading())
                .build();

        grab4PPG = follower.pathBuilder() // Move to the fourth grabbing pose from the third grabbing pose
                .addPath(new BezierLine(PPG3Pose, PPG4Pose))
                .setLinearHeadingInterpolation(PPG3Pose.getHeading(), PPG4Pose.getHeading())
                .build();

        // Move to the scoring pose from the last artifact pickup pose
        scorePPG = follower.pathBuilder()
                .addPath(new BezierLine(PPG4Pose, scorePose))
                .setLinearHeadingInterpolation(PPG4Pose.getHeading(), scorePose.getHeading())
                .build();

        // Move to the final pose from the scoring pose
        finalPosition = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finalPose.getHeading())
                .build();
    }

    public void buildPathsPGP() {
        // basically just plotting the points for the lines that score the PGP pattern

        initialScorePGP = follower.pathBuilder() // Move to the scoring pose from the stating pose
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPGP = follower.pathBuilder() // Move to the grabbing pose from the scoring pose
                .addPath(new BezierLine(scorePose, PGPPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), PGPPose.getHeading())
                .build();

        grab2PGP = follower.pathBuilder() // Move to the second grabbing pose from the first grabbing pose
                .addPath(new BezierLine(PGPPose, PGP2Pose))
                .setLinearHeadingInterpolation(PGPPose.getHeading(), PGP2Pose.getHeading())
                .build();

        grab3PGP = follower.pathBuilder() // Move to the third grabbing pose from the second grabbing pose
                .addPath(new BezierLine(PGP2Pose, PGP3Pose))
                .setLinearHeadingInterpolation(PGP2Pose.getHeading(), PGP3Pose.getHeading())
                .build();

        grab4PGP = follower.pathBuilder() // Move to the fourth grabbing pose from the third grabbing pose
                .addPath(new BezierLine(PGP3Pose, PGP4Pose))
                .setLinearHeadingInterpolation(PGP3Pose.getHeading(), PGP4Pose.getHeading())
                .build();

        // Move to the scoring pose from the last artifact pickup pose
        scorePGP = follower.pathBuilder()
                .addPath(new BezierLine(PGP4Pose, scorePose))
                .setLinearHeadingInterpolation(PGP4Pose.getHeading(), scorePose.getHeading())
                .build();

        // Move to the final pose from the scoring pose
        finalPosition = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finalPose.getHeading())
                .build();
    }

    public void buildPathsGPP() {
        // basically just plotting the points for the lines that score the GPP pattern

        initialScoreGPP = follower.pathBuilder() // Move to the scoring pose from the stating pose
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabGPP = follower.pathBuilder() // Move to the grabbing pose from the scoring pose
                .addPath(new BezierLine(scorePose, GPPPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), GPPPose.getHeading())
                .build();

        grab2GPP = follower.pathBuilder() // Move to the second grabbing pose from the first grabbing pose
                .addPath(new BezierLine(GPPPose, GPP2Pose))
                .setLinearHeadingInterpolation(GPPPose.getHeading(), GPP2Pose.getHeading())
                .build();

        grab3GPP = follower.pathBuilder() // Move to the third grabbing pose from the second grabbing pose
                .addPath(new BezierLine(GPP2Pose, GPP3Pose))
                .setLinearHeadingInterpolation(GPP2Pose.getHeading(), GPP3Pose.getHeading())
                .build();

        grab4GPP = follower.pathBuilder() // Move to the fourth grabbing pose from the third grabbing pose
                .addPath(new BezierLine(GPP3Pose, GPP4Pose))
                .setLinearHeadingInterpolation(GPP3Pose.getHeading(), GPP4Pose.getHeading())
                .build();

        // Move to the scoring pose from the last artifact pickup pose
        scoreGPP = follower.pathBuilder()
                .addPath(new BezierLine(GPP4Pose, scorePose))
                .setLinearHeadingInterpolation(GPP4Pose.getHeading(), scorePose.getHeading())
                .build();

        // Move to the final pose from the scoring pose
        finalPosition = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, finalPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), finalPose.getHeading())
                .build();
    }

    //below is the state machine or each pattern

    public void updateStateMachinePPG() {
        switch (pathStatePPG) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(initialScorePPG);
                setpathStatePPG(1); // Call the setter method
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Shoot the artifacts since we are in shooting pose
                    shootArtifactsLeft();
                    // Move to the first artifact pickup location from the scoring position
                    follower.followPath(grabPPG);
                    setpathStatePPG(2); // Call the setter method
                }
                break;
            case 2:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn on the initial intake stage
                    initialIntakeArtifacts();
                    // Move to the second artifact pickup location from the initial pickup location
                    follower.followPath(grab2PPG);
                    setpathStatePPG(3); // Call the setter method
                }
                break;
            case 3:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn off first stage intake
                    intakeArtifacts();
                    // Move to the third artifact pickup location from the second pickup location
                    follower.followPath(grab3PPG);
                    setpathStatePPG(4); // Call the setter method
                }
                break;
            case 4:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Move to the fourth artifact pickup location from the third pickup location
                    follower.followPath(grab4PPG);
                    setpathStatePPG(5); // Call the setter method
                }
                break;
            case 5:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn off intake
                    stopIntakeArtifacts();
                    // Move to the shooting pose from the fourth pickup location
                    follower.followPath(scorePPG);
                    setpathStatePPG(6); // Call the setter method
                }
                break;
            case 6:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Shoot the artifacts
                    if(hue>350) {
                        shootArtifactsLeft();
                    } else {
                        shootArtifactsRight();
                    }
                    // Go to the final position
                    follower.followPath(finalPosition);
                    setpathStatePPG(-1); //set it to -1 so it stops the state machine execution
                }
                break;
        }
    }


    public void updateStateMachinePGP() {
        switch (pathStatePGP) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(initialScorePGP);
                setpathStatePGP(1); // Call the setter method
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Shoot the artifacts since we are in shooting pose
                    shootArtifactsLeft();
                    // Move to the first artifact pickup location from the scoring position
                    follower.followPath(grabPGP);
                    setpathStatePGP(2); // Call the setter method
                }
                break;
            case 2:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn on the initial intake stage
                    initialIntakeArtifacts();
                    // Move to the second artifact pickup location from the initial pickup location
                    follower.followPath(grab2PGP);
                    setpathStatePGP(3); // Call the setter method
                }
                break;
            case 3:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn off first stage intake
                    intakeArtifacts();
                    // Move to the third artifact pickup location from the second pickup location
                    follower.followPath(grab3PGP);
                    setpathStatePGP(4); // Call the setter method
                }
                break;
            case 4:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Move to the fourth artifact pickup location from the third pickup location
                    follower.followPath(grab4PGP);
                    setpathStatePGP(5); // Call the setter method
                }
                break;
            case 5:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn off intake
                    stopIntakeArtifacts();
                    // Move to the shooting pose from the fourth pickup location
                    follower.followPath(scorePGP);
                    setpathStatePGP(6); // Call the setter method
                }
                break;
            case 6:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Shoot the artifacts
                    if(hue > 350) {
                        shootArtifactsRight();
                    } else {
                        shootArtifactsLeft();
                    }
                    // Go to the final position
                    follower.followPath(finalPosition);
                    setpathStatePGP(-1); //set it to -1 so it stops the state machine execution
                }
                break;
        }
    }


    public void updateStateMachineGPP() {
        switch (pathStateGPP) {
            case 0:
                // Move to the scoring position from the start position
                follower.followPath(initialScoreGPP);
                setpathStateGPP(1); // Call the setter method
                break;
            case 1:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Shoot the artifacts since we are in shooting pose
                    shootArtifactsLeft();
                    // Move to the first artifact pickup location from the scoring position
                    follower.followPath(grabGPP);
                    setpathStateGPP(2); // Call the setter method
                }
                break;
            case 2:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn on the initial intake stage
                    initialIntakeArtifacts();
                    // Move to the second artifact pickup location from the initial pickup location
                    follower.followPath(grab2GPP);
                    setpathStateGPP(3); // Call the setter method
                }
                break;
            case 3:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn off first stage intake
                    intakeArtifacts();
                    // Move to the third artifact pickup location from the second pickup location
                    follower.followPath(grab3GPP);
                    setpathStateGPP(4); // Call the setter method
                }
                break;
            case 4:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Move to the fourth artifact pickup location from the third pickup location
                    follower.followPath(grab4GPP);
                    setpathStateGPP(5); // Call the setter method
                }
                break;
            case 5:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Turn off intake
                    stopIntakeArtifacts();
                    // Move to the shooting pose from the fourth pickup location
                    follower.followPath(scoreGPP);
                    setpathStateGPP(6); // Call the setter method
                }
                break;
            case 6:
                // Wait until we have passed all path constraints
                if (!follower.isBusy()) {

                    // Shoot the artifacts
                    shootArtifactsLeft();
                    // Go to the final position
                    follower.followPath(finalPosition);
                    setpathStateGPP(-1); //set it to -1 so it stops the state machine execution
                }
                break;
        }
    }

    // Setter methods for pathState variables placed at the class level
    void setpathStatePPG(int newPathState) {
        this.pathStatePPG = newPathState;
    }

    void setpathStatePGP(int newPathState) {
        this.pathStatePGP = newPathState;
    }

    void setpathStateGPP(int newPathState) {
        this.pathStateGPP = newPathState;
    }


    /**
     * start the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        // ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        // if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
        //     exposureControl.setMode(ExposureControl.Mode.Manual);
        //     sleep(50);
        // }
        // exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
        // sleep(20);
        // GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        // gainControl.setGain(gain);
        // sleep(20);
    }
}