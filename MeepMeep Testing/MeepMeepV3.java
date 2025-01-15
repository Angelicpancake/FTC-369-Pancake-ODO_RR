package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepV3 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(400);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -62, 3*Math.PI/2))
                .strafeTo(new Vector2d(-4,-30))
                .lineToY(-40)

                //line up to get first sample
                .strafeToLinearHeading(new Vector2d(35,-40), 0)
                .strafeToConstantHeading(new Vector2d(35,-26))

                //get first and drop it in observation
                .strafeToConstantHeading(new Vector2d(40,-26))
                .strafeToConstantHeading(new Vector2d(40,-55))

                // go to second and grab and drop
                .strafeToConstantHeading(new Vector2d(45,-26))
                .strafeToConstantHeading(new Vector2d(55,-26))
                .strafeToConstantHeading(new Vector2d(55,-55))




//                //35, - 58
//                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2)
//                .strafeToLinearHeading(new Vector2d(0, -33.5), 3*Math.PI/2)
//
//                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2)
//                .strafeToLinearHeading(new Vector2d(4, -33.5), 3*Math.PI/2)
//
//                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2)
//                .strafeToLinearHeading(new Vector2d(8, -33.5), 3*Math.PI/2)
//
//                .strafeToLinearHeading(new Vector2d(37, -60), Math.PI/2)
//                .strafeToLinearHeading(new Vector2d(12, -33.5), 3*Math.PI/2)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}