package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Vision;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Subsystem;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LazyServo;
import org.firstinspires.ftc.teamcode.robots.lebot2.util.LimelightStream;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;


@Config(value = "Bumblebee_Vision")
public class Vision implements Subsystem {
    public boolean isRedAlliance = false;

    // ==================== HARDWARE ====================
    private final Limelight3A limeLight;
    private final LazyServo tilt;

  
    // ==================== SERVO STATES ====================
    public static int tiltUpTicks = 0;
    public static int tiltDownTicks = 0;
    private static final double tiltDownDegrees = 0.0;
    private static final double tiltUpDegrees = 0.0;
    public static long timeToSettle_ms = 500L;
    private long switchRequested_ms = 0L;

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
    private boolean tiltPending = false;
    // ==================== HIVE STATES ====================
    private final HiveTracker redHive = new HiveTracker(true, this);
    private final HiveTracker blueHive = new HiveTracker(false, this);

    private final List<LLResultTypes.FiducialResult> redFiducials = new ArrayList<>();
    private final List<LLResultTypes.FiducialResult> blueFiducials = new ArrayList<>();


    // ==================== LOCALIZATION VARIABLES ====================
    private double tx = 0; 
    private double ty = 0; 
    private double ta = 0;
    private boolean hasBotPose = false;
    private double robotX = 0.0, robotY = 0.0, robotHeading = 0.0 ;
    private Pose3D botPose = null;
    private double lastTimestamp = 0.0;


    // ==================== VISION PIPELINES ====================
    public static int pollenTrackingPipeline = 2;
    public static int dummyPipeline = 3;
    public static int redAudienceUpBlueAudienceUp = 4;
    public static int redAudienceUpBlueAudienceDown = 5;
    public static int redAudienceDownBlueAudienceUp = 6;
    public static int redAudienceDownBlueAudienceDown = 7;

    public static enum Pipeline{
        
        POLLEN(pollenTrackingPipeline, false),
        DUMMY(dummyPipeline,false),
        RED_AUDIENCE_UP_BLUE_AUDIENCE_UP(redAudienceUpBlueAudienceUp, true),
        RED_AUDIENCE_UP_BLUE_AUDIENCE_DOWN(redAudienceUpBlueAudienceDown, true),
        RED_AUDIENCE_DOWN_BLUE_AUDIENCE_UP(redAudienceDownBlueAudienceUp, true),
        RED_AUDIENCE_DOWN_BLUE_AUDIENCE_DOWN(redAudienceDownBlueAudienceDown, true);

        public final int id;
        public final boolean isLocalizer;
        Pipeline(int id, boolean isLocalizer){
          this.id = id;
          this.isLocalizer = isLocalizer;
        }
    }

    // Ignore frames until the initial servo move and pipeline switch complete.
    private Pipeline pipeline = Pipeline.DUMMY;
    private Pipeline requestedPipeline;
    private boolean pipelineRequested;

    // ==================== DASHBOARD ====================
    private LimelightStream limelightStream = null;
    public static boolean ENABLE_DASHBOARD_STREAM = false;  // Toggle via dashboard config
    public static int STREAM_FPS = 5;  // Target FPS for dashboard streaming (keep low to reduce lag)
    public static double STREAM_SCALE = 0.5;  // Scale factor forDistance calculations require botpose. dashboard image (0.25-1.0)


    public static enum Behavior{
      LOCALIZING,
      POLLEN_TRACKING,
    }

    public Behavior behavior;

    public Vision(HardwareMap hardwareMap){
        limeLight  = hardwareMap.get(Limelight3A.class, "limeLight");
        limeLight.start();
        tilt = new LazyServo(hardwareMap, "tilt");
        setPipeline(Pipeline.RED_AUDIENCE_UP_BLUE_AUDIENCE_DOWN);
        behavior = Behavior.LOCALIZING;
    }


    @Override
    public void readSensors() {
    }

    @Override
    public void calc(Canvas fieldOverlay) {

        if(pipelineRequested && !tiltPending  && System.nanoTime()/1_000_000 - switchRequested_ms >= timeToSettle_ms){
            if(limeLight.pipelineSwitch(requestedPipeline.id)){
                pipeline = requestedPipeline;
                tiltState = pipeline.isLocalizer ? TiltState.TILT_UP : TiltState.TILT_DOWN;
                pipelineRequested = false;    
            }

        }

        LLResult result = limeLight.getLatestResult();

        switch (behavior){
            case LOCALIZING:
                if(pipelineRequested || !pipeline.isLocalizer || result == null || !result.isValid()
                        || result.getPipelineIndex() != pipeline.id){
                    hasBotPose = false;
                    botPose = null;
                    redHive.update(Collections.emptyList());
                    blueHive.update(Collections.emptyList());
                    break;
                }
                if(result.getTimestamp() != lastTimestamp){
                    lastTimestamp = result.getTimestamp();
                    hasBotPose = false;
                    botPose = null;

                    // Clear fiducial sets
                    redFiducials.clear();
                    blueFiducials.clear();

                    // Sort fiducials by alliance
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        if(HiveTracker.isBlueTag(tag.getFiducialId())) blueFiducials.add(tag);
                        else if(HiveTracker.isRedTag(tag.getFiducialId())) redFiducials.add(tag);
                    }

                    // Update each Hive state
                    redHive.update(redFiducials);
                    blueHive.update(blueFiducials);

                    if(redHive.getHiveState() == HiveTracker.HiveState.UNKNOWN || blueHive.getHiveState() == HiveTracker.HiveState.UNKNOWN) break;

                    Pipeline toUse;
                    if(blueHive.getHiveState() == HiveTracker.HiveState.AUDIENCE_SIDE_UP &&  redHive.getHiveState() == HiveTracker.HiveState.AUDIENCE_SIDE_UP ){
                        toUse = Pipeline.RED_AUDIENCE_UP_BLUE_AUDIENCE_UP;
                    }else if(blueHive.getHiveState() == HiveTracker.HiveState.REAR_SIDE_UP &&  redHive.getHiveState() == HiveTracker.HiveState.REAR_SIDE_UP ){
                        toUse = Pipeline.RED_AUDIENCE_DOWN_BLUE_AUDIENCE_DOWN;
                    }else if(blueHive.getHiveState() == HiveTracker.HiveState.AUDIENCE_SIDE_UP && redHive.getHiveState() == HiveTracker.HiveState.REAR_SIDE_UP){
                        toUse = Pipeline.RED_AUDIENCE_DOWN_BLUE_AUDIENCE_UP;
                    }else{
                        toUse = Pipeline.RED_AUDIENCE_UP_BLUE_AUDIENCE_DOWN;
                    }

                    if( pipeline != toUse){
                      setPipeline(toUse);
                      break;
                    }

                    tx = result.getTx();
                    ty = result.getTy();
                    ta = result.getTa();

                    botPose = result.getBotpose_MT2();

                    // Extract MT2 pose data
                    if (botPose != null ) {
                        hasBotPose = true;
                        robotX = botPose.getPosition().x;
                        robotY = botPose.getPosition().y;
                        robotHeading = Math.toRadians(botPose.getOrientation().getYaw());
                    } else {
                        hasBotPose = false;
                    }
                }

            break;

            case POLLEN_TRACKING:
            break;

        }

    }

    @Override
    public void act() {
        tilt.flush();
        if(pipelineRequested && tiltPending){
            switchRequested_ms = System.nanoTime() / 1_000_000;
            tiltPending = false;
        }

    }
    
    // ==================== PIPELINE AND SERVO POSITION SETTER ====================
    
    /*
     * Set limelight pipeline. Reject switch if requested pipeline is already running or switching to pipeline is in progress
    */
    private void setPipeline(Pipeline requestedPipeline){
        if((!pipelineRequested && pipeline == requestedPipeline) || requestedPipeline == Pipeline.DUMMY) return;
        if(pipelineRequested && this.requestedPipeline == requestedPipeline) return;
        hasBotPose = false;
        botPose = null;
        behavior = requestedPipeline.isLocalizer ? Behavior.LOCALIZING : Behavior.POLLEN_TRACKING;

        // Don't tilt if you are switching bw localization pipelines
        if(pipeline == Pipeline.DUMMY || tiltState != (requestedPipeline.isLocalizer ? TiltState.TILT_UP : TiltState.TILT_DOWN)){
          setServo(requestedPipeline);
          tiltPending = true;
          pipeline = Pipeline.DUMMY;
          limeLight.pipelineSwitch(dummyPipeline);
        }

        this.requestedPipeline = requestedPipeline;
        pipelineRequested = true;
    }

    /*
    * Queue servoPosition based on requested pipeline
    */
    private void setServo(Pipeline requestedPipeline){
        if(requestedPipeline.isLocalizer){
            tilt.setPosition(servoNormalize(TiltState.TILT_UP.ticks));
        }else if(requestedPipeline == Pipeline.POLLEN) {
            tilt.setPosition(servoNormalize(TiltState.TILT_DOWN.ticks));
        }

        tiltState = TiltState.MOVING;
    }


    public void setBehavior(Behavior behavior){
        this.behavior = behavior;
    }

    /**
     * Update the Limelight with current robot orientation for MegaTag2 localization.
     * Call this before calc() each loop with a heading aligned to the field map.
     *
     * @param yawDegrees Field-aligned robot heading in degrees
     */
    public void updateRobotOrientation(double yawDegrees) {
        limeLight.updateRobotOrientation(yawDegrees);
    }

    // ==================== LOCALIZATION VALUES ====================

    /**
     * @return If we have a valid botpose for field localization
     */
    public boolean hasBotPose() {
        return hasBotPose ;
    }

    /**
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

    /**
     * Get target area.
     *
     * @return ta as percentage (0-100)
     */
    public double getTa() {
        return ta;
    }


    // ==================== POSITION GETTERS ====================
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
     * Get bot pose from AprilTag localization.
     *
     * @return Pose3D or null if not available
     */
    public Pose3D getBotPose() {
        return hasBotPose() ? botPose : null;
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
        return ((double) pulse - 750.0) / 1500.0; //convert mr servo controller pulse width to double on _0 - 1 scale
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
      limeLight.stop();
      hasBotPose = false;
      botPose = null;
    }

    @Override
    public void resetStates() {
    }

}
