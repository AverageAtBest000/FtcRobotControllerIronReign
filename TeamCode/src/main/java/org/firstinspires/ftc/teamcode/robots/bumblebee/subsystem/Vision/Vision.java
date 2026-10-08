package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyServo;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LimelightStream;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set; 


@Config(value = "Bumblebee_Vision")
public class Vision implements Subsystem {

    class AprilTagPositions{
        public Pose3D audienceSideUp;
        public Pose3D rearSideUp;
        public AprilTagPositions(Pose3D audienceSideUp, Pose3D rearSideUp){
            this.audienceSideUp = audienceSideUp;
            this.rearSideUp = rearSideUp;
        }
    }

    // ==================== HARDWARE ====================
    private final Limelight3A limeLight;
    private final LazyServo tilt;

  
    // ==================== SERVO STATES ====================
    public static int tiltUpTicks = 0;
    public static int tiltDownTicks = 0;
    private static final double tiltDownDegrees = 0.0;
    private static final double tiltUpDegrees = 0.0;
    public enum TiltState{
      TILT_UP(tiltUpTicks, tiltUpDegrees),
      TILT_DOWN(tiltDownTicks, tiltDownDegrees),
        MOVING(-1, -1.0);

      public final int ticks;
      public final double degrees;
      TiltState(int ticks, double degrees) {
      this.ticks = ticks;
      this.degrees = degrees;
      }

    }

    public TiltState tiltState = TiltState.TILT_UP;

    // ==================== HIVE STATES ====================
    private HiveTracker redHive = new HiveTracker(true, this);
    private HiveTracker blueHive = new HiveTracker(false, this);

    private List<LLResultTypes.FiducialResult> redFiducials = new ArrayList<>();
    private List<LLResultTypes.FiducialResult> blueFiducials = new ArrayList<>();

    // ==================== APRILTAG POSES ====================
    HashMap<Integer, Pose3D> RED_TAGS = new HashMap<>();
    HashMap<Integer, Pose3D> BLUE_TAGS = new HashMap<>();

    {
        RED_TAGS.put(30, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        RED_TAGS.put(31, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        RED_TAGS.put(32, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        RED_TAGS.put(33, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        RED_TAGS.put(34, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        RED_TAGS.put(35, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        RED_TAGS.put(36, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        RED_TAGS.put(37, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));

        BLUE_TAGS.put(38, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        BLUE_TAGS.put(39, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        BLUE_TAGS.put(40, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        BLUE_TAGS.put(41, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        BLUE_TAGS.put(42, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        BLUE_TAGS.put(43, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        BLUE_TAGS.put(44, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
        BLUE_TAGS.put(45, new Pose3D(
                new Position(DistanceUnit.METER, 0.0, 0.0, 0.0, 0L),
                new YawPitchRollAngles(AngleUnit.DEGREES, 0.0, 0.0, 0.0, 0L)));
    }

    // ==================== LOCALIZATION VARIABLES ====================
    private double tx = 0; 
    private double ty = 0; 
    private double ta = 0; 

    private double distanceToGoal = 0;

    private boolean hasValidTarget = false;
    private boolean hasBotPose = false;

    private Pose3D mt2Pose = null;
    boolean hasMT2Pose = false;
    private double mt2X = 0, mt2Y = 0, mt2Heading = 0;

    private double robotX = 0.0, robotY = 0.0, robotHeading = 0.0 ;
    private Pose3D botPose = null;

    private double lastTimestamp = 0.0;

    // ==================== VISION PIPELINES ====================
    public enum Pipeline{
        
        POLLEN(1),
        APRILTAG(2);

        public final int id;
        Pipeline(int id){this.id = id;}
    }

    Pipeline pipeline = Pipeline.APRILTAG;

    // ==================== DASHBOARD ====================
    private LimelightStream limelightStream = null;
    public static boolean ENABLE_DASHBOARD_STREAM = false;  // Toggle via dashboard config
    public static int STREAM_FPS = 5;  // Target FPS for dashboard streaming (keep low to reduce lag)
    public static double STREAM_SCALE = 0.5;  // Scale factor for dashboard image (0.25-1.0)


    public Vision(HardwareMap hardwareMap){
        limeLight  = hardwareMap.get(Limelight3A.class, "limeLight");
        limeLight.start();
        tilt = new LazyServo(hardwareMap, "tilt");
    }


    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay) {
      
        LLResult result = limeLight.getLatestResult();

        switch (pipeline){
            case APRILTAG:
                if(result != null && result.getTimestamp() != lastTimestamp){
                    lastTimestamp = result.getTimestamp();

                    // Clear fiducial sets
                    redFiducials.clear();
                    blueFiducials.clear();

                    // Sort fiducials by alliance
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        if(HiveTracker.isBlueTag(tag.getFiducialId())) blueFiducials.add(tag);
                        else redFiducials.add(tag);
                    }

                    // Update each Hive state
                    redHive.update(redFiducials);
                    blueHive.update(blueFiducials);



                }
            break;

            case POLLEN:

            break;
        }

    }

    @Override
    public void act() {

    }
    
    // ==================== PIPELINE SETTER ====================
    
    /*
     * Set limelight pipeline 
    */
    public void setPipeline(Pipeline pipeline){
        this.pipeline = pipeline;
        limeLight.pipelineSwitch(pipeline.id);
    }


    // ==================== SERVO POSITION SETTERS ====================

    /*
     * Set servo position up
    */
    public void setTiltUp(){
        tilt.setPosition(servoNormalize(TiltState.TILT_UP.ticks));
        tiltState = TiltState.TILT_UP;
    }
   
    /*
     * Set servo position down
    */
    public void setTiltDown(){
        tilt.setPosition(servoNormalize(TiltState.TILT_DOWN.ticks));
        tiltState = TiltState.TILT_DOWN;
    }
    

    // ==================== LOCALIZATION VALUES====================
    /*
     * Check if Limelight has a valid target in view.
     */
    public boolean hasTarget() {
        return hasValidTarget;
    }

    /*
     * Check if we have a valid botpose for field localization.
     * Distance calculations require botpose.
     */
    public boolean hasBotPose() {
        return hasBotPose;
    }

    /*
     * Get horizontal offset to target.
     *
     * @return tx in degrees. Negative = target is left, Positive = right
     */
    public double getTx() {
        return tx;
    }

    /**
     * Get vertical offset to target.
     *
     * @return ty in degrees. Used for distance calculation.
     */
    public double getTy() {
        return ty;
    }

    /*
     * Get target area.
     *
     * @return ta as percentage (0-100)
     */
    public double getTa() {
        return ta;
    }


    // ==================== TARGET DISTANCE ESTIMATION ====================
    // For tracking a handheld AprilTag (not field-fixed goal)
    // Distance estimated from target area using inverse square law: distance = k / sqrt(ta)
    public static double TARGET_DISTANCE_K = 30.0;  // Calibration constant (tune with known distance)

    /**
     * Get estimated distance to the current target based on apparent size.
     * Uses the inverse square law: distance ∝ 1/sqrt(area).
     * Calibrate TARGET_DISTANCE_K by measuring actual distance when ta is known.
     *
     * @return Estimated distance in inches (0 if no valid target)
     */
    public double getTargetDistanceInches() {
        if (!hasValidTarget || ta <= 0) {
            return 0;
        }
        // Inverse square law: as distance doubles, area quarters
        // So distance = k / sqrt(ta)
        return TARGET_DISTANCE_K / Math.sqrt(ta);
    }

    /**
     * Get calculated distance to goal.
     * Uses botpose field localization + known goal positions.
     *
     * @return Distance in meters (0 if no valid botpose)
     */
    public double getDistanceToGoal() {
        return distanceToGoal;
    }

    /**
     * Get robot's X position on field (from botpose).
     * @return X in meters from field center
     */
    public double getRobotX() {
        return robotX;
    }

    /**
     * Get robot's Y position on field (from botpose).
     * @return Y in meters from field center
     */
    public double getRobotY() {
        return robotY;
    }

    /**
     * Get robot's heading from botpose.
     * @return Heading in radians
     */
    public double getRobotHeading() {
        return robotHeading;
    }

    /**
     * Get bot pose from AprilTag localization.
     *
     * @return Pose3D or null if not available
     */
    public Pose3D getBotPose() {
        return botPose;
    }

    // ==================== DASHBOARD STREAMING ====================

    /**
     * Start streaming frames to FTC Dashboard.
     * Fetches frames from Limelight's MJPEG stream on a background thread.
     */
    public void startDashboardStream() {
        if (limelightStream == null) {
            limelightStream = new LimelightStream();
        }
        limelightStream.setTargetFps(STREAM_FPS);
        limelightStream.start();
    }

    /**
     * Stop streaming frames to FTC Dashboard.
     */
    public void stopDashboardStream() {
        if (limelightStream != null) {
            limelightStream.stop();
        }
    }

    /**
     * Get a new frame for dashboard display, scaled down to reduce bandwidth.
     * Returns null if streaming is not active, no frame available, or no new frame since last call.
     * Only returns each frame once to avoid redundant dashboard updates.
     *
     * @return Scaled Bitmap frame or null
     */
    public Bitmap getDashboardFrame() {
        if (limelightStream == null) return null;

        Bitmap frame = limelightStream.getNewFrame();
        if (frame == null) return null;

        // Scale down to reduce bandwidth and CPU load
        double scale = Math.max(0.1, Math.min(1.0, STREAM_SCALE));
        if (scale < 1.0) {
            int newWidth = (int) (frame.getWidth() * scale);
            int newHeight = (int) (frame.getHeight() * scale);
            if (newWidth > 0 && newHeight > 0) {
                try {
                    Bitmap scaled = Bitmap.createScaledBitmap(frame, newWidth, newHeight, true);
                    return scaled;
                } catch (Exception e) {
                    // If scaling fails, return original
                    return frame;
                }
            }
        }
        return frame;
    }

    /**
     * Check if there's a new frame available (not yet sent to dashboard).
     */
    public boolean hasNewDashboardFrame() {
        return limelightStream != null && limelightStream.hasNewFrame();
    }

    /**
     * Check if dashboard streaming is currently active.
     */
    public boolean isDashboardStreamActive() {
        return limelightStream != null && limelightStream.isRunning();
    }

    /**
     * Get dashboard stream status for telemetry.
     */
    public String getDashboardStreamStatus() {
        if (limelightStream == null) return "Not initialized";
        return limelightStream.getStatus();
    }

    public static double servoNormalize(int pulse) {
        double normalized = (double) pulse;
        return (normalized - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
    }
  
    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        return Collections.emptyMap();
    }

    @Override
    public String getTelemetryName() {
        return "Vision";
    }

    @Override
    public void stop() {
      stopDashboardStream();
    }

    @Override
    public void resetStates() {
    }

}

