package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.roadrunner.Pose2d;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.noahbres.meepmeep.MeepMeep;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
//
//public class MeepMeepTesting {
//    public static void main(String[] args) {
//        MeepMeep meepMeep = new MeepMeep(1000);
//
//        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
//                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
//                .build();
//
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -60, 3*Math.PI/2))
//                .lineToY(-35)  // Drive forward
//                // attach specimen
//                .lineToY(-40)  // Move back a bit
//
//                //spline to go to the three smaples
//                .setTangent(0)
//                .lineToX(20)
//                .splineToLinearHeading(new Pose2d(45, -7, 3*Math.PI/2), Math.PI/3)
//
//
//
//
//                //push first sample down to obervation zone
//                .setTangent(Math.PI/2)
//                .lineToY(-55)
//
//
//                // go to second sample
//                .lineToY(-10)
//                .setTangent(0)
//                .lineToX(56)
//                .setTangent(Math.PI/2)
//                .lineToY(-55)
//
//
//
//                //diagonal to get out of obervation zone and get ready to recive specimen
//                .setTangent(-0.25)
//                .lineToXLinearHeading(36,Math.PI/2)
//
//
//
//
//                //clip trial #1
//                .setTangent(Math.PI/2)
//                .lineToYLinearHeading(-60, Math.PI/2)
//                .setTangent(2.4)
//                .lineToXLinearHeading(7, 3*Math.PI/2)
//                .setTangent(2.6)
//                .lineToXLinearHeading(36, Math.PI/2)
//                .setTangent(Math.PI/2)
//                .lineToY(-60)
//
//                //clip #2
//                .setTangent(2.45)
//                .lineToXLinearHeading(5, 3*Math.PI/2)
//                .setTangent(2.7)
//                .lineToXLinearHeading(36, Math.PI/2)
//                .setTangent(Math.PI/2)
//                .lineToY(-60)
//
//                //clip #3
//                .setTangent(2.45)
//                .lineToXLinearHeading(3, 3*Math.PI/2)
//
//                //park - enable if extra time left
//                //.setTangent(2.6)
//                //.lineToXLinearHeading(48, Math.PI/2)
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//                .build());
//
//        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
//                .setDarkMode(false)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(myBot)
//                .start();
//    }
//}