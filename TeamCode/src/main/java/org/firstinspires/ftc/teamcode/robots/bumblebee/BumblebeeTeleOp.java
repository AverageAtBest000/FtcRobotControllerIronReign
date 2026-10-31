package org.firstinspires.ftc.teamcode.robots.bumblebee;



import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Bumblebee", group = "Bumblebee")
public class BumblebeeTeleOp extends OpMode {

    private Robot robot;
    private DriverControls driverControls;
    
    @Override
    public void init(){
      robot = new Robot(hardwareMap);
      driverControls = new DriverControls(robot, gamepad1);
    }

    @Override 
    public void loop(){
        robot.readSensors();
        driverControls.update();
        robot.calc(new Canvas());
        robot.act();
    }

    @Override
    public void stop(){
      robot.stop();
    }  


}
