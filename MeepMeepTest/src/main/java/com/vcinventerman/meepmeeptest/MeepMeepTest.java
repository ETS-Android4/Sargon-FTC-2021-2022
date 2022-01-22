package com.vcinventerman.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {

    public static final double robotRadius = 8.375; // Length from center of robot to back


    public static final double armHighOffset = 18;
    public static final double armMedOffset = 26;
    public static final double armLowOffset = 27.5;

    public static final double armHighSquareOffset = Math.sqrt(0.5 * Math.pow(armHighOffset, 2));
    public static final double armMedSquareOffset = Math.sqrt(0.5 * Math.pow(armMedOffset, 2));
    public static final double armLowSquareOffset = Math.sqrt(0.5 * Math.pow(armLowOffset, 2));



    public static final double redDuckStartingPoseX = -3 * 12;
    public static final double redDuckStartingPoseY = (-6 * 12) + robotRadius;
    public static final double redDuckStartingPoseHeading = Math.toRadians(90);
    public static final Pose2d redDuckStartingPose =
            new Pose2d(redDuckStartingPoseX, redDuckStartingPoseY, redDuckStartingPoseHeading);

    public static final double redWarehouseStartingPoseX = 1 * 12;
    public static final double redWarehouseStartingPoseY = (-6 * 12) + robotRadius;
    public static final double redWarehouseStartingPoseHeading = Math.toRadians(90);
    public static final Pose2d redWarehouseStartingPose =
            new Pose2d(redWarehouseStartingPoseX, redWarehouseStartingPoseY, redWarehouseStartingPoseHeading);

    public static final double blueDuckStartingPoseX = -3 * 12;
    public static final double blueDuckStartingPoseY = (6 * 12) - robotRadius;
    public static final double blueDuckStartingPoseHeading = Math.toRadians(270);
    public static final Pose2d blueDuckStartingPose =
            new Pose2d(blueDuckStartingPoseX, blueDuckStartingPoseY, blueDuckStartingPoseHeading);

    public static final double blueWarehouseStartingPoseX = 1 * 12;
    public static final double blueWarehouseStartingPoseY = (6 * 12) - robotRadius;
    public static final double blueWarehouseStartingPoseHeading = Math.toRadians(270);
    public static final Pose2d blueWarehouseStartingPose =
            new Pose2d(blueWarehouseStartingPoseX, blueWarehouseStartingPoseY, blueWarehouseStartingPoseHeading);


    public static final double redShippingHubX = -1 * 12;
    public static final double redShippingHubY = -2 * 12;
    public static final Vector2d redShippingHubPos = new Vector2d(redShippingHubX, redShippingHubY);

    public static final double blueShippingHubX = -1 * 12;
    public static final double blueShippingHubY = 2 * 12;
    public static final Vector2d blueShippingHubPos = new Vector2d(blueShippingHubX, blueShippingHubY);

    public static final double sharedShippingHubX = 4 * 12;
    public static final double sharedShippingHubY = 0;
    public static final Vector2d sharedShippingHubPos = new Vector2d(sharedShippingHubX, sharedShippingHubY);


    public static final double redDuckSpinnerX = -6 * 12;
    public static final double redDuckSpinnerY = -6 * 12;
    public static final Vector2d redDuckSpinnerPos = new Vector2d(redDuckSpinnerX, redDuckSpinnerY);

    public static final double blueDuckSpinnerX = -6 * 12;
    public static final double blueDuckSpinnerY = 6 * 12;
    public static final Vector2d blueDuckSpinnerPos = new Vector2d(blueDuckSpinnerX, blueDuckSpinnerY);






    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800).setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK);

        Vector2d shippingHubPos = new Vector2d(redShippingHubX - armMedOffset, redShippingHubY);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.2629039053496918, 30, 4.358889102935791, Math.toRadians(180), 10.71)
                .setDimensions(12, 17.875)
                /*.followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(1 * 12, (6 * 12) - 8.375, Math.toRadians(270)))
                                .strafeTo(new Vector2d((-1 * 12) + 18.38477631, (2 * 12) + 18.38477631))
                                .turn(Math.toRadians(-45))
                                .lineToLinearHeading(new Pose2d(new Vector2d(1 * 12, (5.5 * 12) - 8.375), Math.toRadians(0))) // Go near wall
                                .strafeLeft((.5 * 12))
                                .forward(2.5 * 12)
                                .build()

                );*/

                // AutoBlueDuck.java
                /*
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(blueDuckStartingPose)
                                .strafeTo(new Vector2d((-4.95 * 12), (4.9 * 12) )) // Blue duck carousel
                                .waitSeconds(4) // Wait for ducks to fall
                                .lineToLinearHeading(new Pose2d((-5 * 12), (2 * 12), 0)) // In line with shipping hub
                                .strafeTo(shippingHubPos) // Approach shipping hub
                                .strafeTo(new Vector2d((-6 * 12) + 8, (3 * 12))) // Return to scoring square
                */

                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(redDuckStartingPose)
                                        .strafeTo(new Vector2d((-4.95 * 12), (-4.9 * 12) )) // Blue duck carousel
                                        .waitSeconds(4) // Wait for ducks to fall
                                        .lineToLinearHeading(new Pose2d((-5 * 12), (-2 * 12), 0)) // In line with shipping hub
                                        .strafeTo(shippingHubPos) // Approach shipping hub
                                        .strafeTo(new Vector2d((-6 * 12) + 8, -(3 * 12))) // Return to scoring square


                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }












}