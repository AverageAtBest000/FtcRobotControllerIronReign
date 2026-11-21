/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.DTPosition;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.PositionCache;
import org.firstinspires.ftc.teamcode.robots.deepthought.util.ExponentialSmoother;

import java.util.LinkedHashMap;
import java.util.Map;


@TeleOp(name="9 PositionLogger test", group="Iterative Opmode")
public class PositionLoggerTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private FtcDashboard dashboard;
    TelemetryPacket packet = new TelemetryPacket();
    int n;
    DTPosition testpos;
    private PositionCache positionCache = new PositionCache(5);
    long lastLoopClockTime, loopTime;

    private double averageLoopTime;
    private ExponentialSmoother loopTimeSmoother;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        loopTimeSmoother = new ExponentialSmoother(0.1);
        DTPosition testread = positionCache.readPose();
        System.out.println("Timestamp: " + testread.getTimestamp());
    }

    @Override
    public void init_loop() {
        
    }


    @Override
    public void start() {
        testpos = new DTPosition();
        testpos.setPose(new Pose2d(5, 6, 3.14));
        positionCache.writePose(testpos, false);
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void loop() {

        packet = new TelemetryPacket();

        n = positionCache.update(testpos, false);

        Map<String, Object> opModeTelemetryMap = new LinkedHashMap<>();

        opModeTelemetryMap.put("Average Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        opModeTelemetryMap.put("Last Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (loopTime * 1e-6), (int) (1 / (loopTime * 1e-9))));
        opModeTelemetryMap.put("cycles", n);
        opModeTelemetryMap.put("last read timestamp", testpos.getTimestamp());
        opModeTelemetryMap.put("testpos x", testpos.getPose().position.x);
        opModeTelemetryMap.put("testpos heading", testpos.getPose().heading.log());
        opModeTelemetryMap.put("testpos y", testpos.getPose().position.y);
        handleTelemetry(opModeTelemetryMap, "stuff", packet);
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
        updateTiming();
    }


    @Override
    public void stop() {
        DTPosition testread = positionCache.readPose();
        System.out.println("Timestamp: " + testread.getTimestamp() );
    }

    private void updateTiming() {
        long loopClockTime = System.nanoTime();
        loopTime = loopClockTime - lastLoopClockTime;
        averageLoopTime = loopTimeSmoother.update(loopTime);
        lastLoopClockTime = loopClockTime;
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet) {
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }
}
