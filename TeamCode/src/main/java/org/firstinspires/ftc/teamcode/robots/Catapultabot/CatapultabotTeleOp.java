package org.firstinspires.ftc.teamcode.robots.Catapultabot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//don't know what this and next line do, but octavio says to use them.
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Catapultabot", group = "Catapultabot")
public class CatapultabotTeleOp extends OpMode {

    private Robot catapultabot;
    private DriverControls controls;

    @Override
    public void init() {
        catapultabot = new Robot(hardwareMap);
        //gamepad1 comes from "extends OpMode"
        controls = new DriverControls(catapultabot, gamepad1);
    }

    @Override
    public void loop() {
        catapultabot.readSensors();
        controls.update();
        catapultabot.calc(new Canvas());
        catapultabot.act();
    }

    //from subsystems apparently (Robot)
    @Override
    public void stop(){
        catapultabot.stop();
    }
}
