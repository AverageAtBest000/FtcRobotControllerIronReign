package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class HiveTracker{
    

    // ==================== HIVE STATES ====================
    public static enum HiveState{
      AUDIENCE_SIDE_UP,
      REAR_SIDE_UP,
      UNKNOWN
    }

    boolean isRedAlliance;
    
    // ==================== POSITION CONFIDENCE ====================
    HiveState hiveState;
    HiveState candidateState;
    private long startTime_ms = System.nanoTime() / 1_000_000;
    public static long timeThreshold_ms = 500L;
    public static long lastSeen_ms = 0L;
    public static double upThreshold = -0.2;
    public static double downThreshold = 0.2;

    private boolean isStale = false;
    public static long isStaleTreshold_ms  = 500L;  
    private long lastStateSet_ms;
    // ==================== TAG INITIAL ORITNTATION ====================
    private enum AprilTagOrientation{
      UP,
      DOWN,
      UNKNOWN
    }

    // ==================== VISION TILT ====================
    public Vision vision; 
    

     // ==================== CONSTRUCTOR ====================
    public HiveTracker(boolean isRedAlliance, Vision vision){
        hiveState = (isRedAlliance) ?  HiveState.AUDIENCE_SIDE_UP: HiveState.REAR_SIDE_UP;
        candidateState = HiveState.UNKNOWN;
        this.vision = vision;
        this.isRedAlliance = isRedAlliance;
    }
    
       
    // ==================== UPDATE HIVE POSITION ====================

  public void update( List<LLResultTypes.FiducialResult> results ){
        long timeNow_ms = System.nanoTime() / 1_000_000;

        // If no tags are observed and staleness threshold has been reached
        if(results.isEmpty() && timeNow_ms - lastSeen_ms >= isStaleTreshold_ms ){
            isStale = true;
            candidateState = HiveState.UNKNOWN;
            startTime_ms = timeNow_ms;
            return;
        }else if(results.isEmpty()){
            // return if staleness threshold has not been reached
            return;
        }

        lastSeen_ms = timeNow_ms;
        isStale = false;

        HiveState observedState = getObservedState(results);

        // keep current candidate if HiveState is unknown
        if(observedState == HiveState.UNKNOWN) return;

        // If the current observedState does not match the candidate
        if(observedState != candidateState){
          // We have a new candiate state - restart the timer
          candidateState = observedState;
          startTime_ms = timeNow_ms;
          return;
        }

        // If we have passed the timeThreshold_ms, candidateState is confirmed stable
        if(timeNow_ms - startTime_ms >= timeThreshold_ms){
          // candidate state is stable - set hiveState to candidate
          hiveState = candidateState;
          lastStateSet_ms = timeNow_ms;
        }
    }


    // ==================== APRILTAG CHECKS   ====================



    private HiveState getObservedState(List<LLResultTypes.FiducialResult> results){

        int audienceUp = 0;
        int rearUp = 0;

        for(LLResultTypes.FiducialResult tag : results){
            double vertical = getYVertical(tag.getTargetPoseCameraSpace());
            AprilTagOrientation orientation = getTagOrientation(vertical);
            HiveState predicted = getPredictedState(orientation, tag);

            if(predicted == HiveState.AUDIENCE_SIDE_UP) audienceUp ++;
            else if (predicted == HiveState.REAR_SIDE_UP) rearUp ++;
        }

        if(rearUp > audienceUp) return HiveState.REAR_SIDE_UP;
        else if( rearUp < audienceUp) return HiveState.AUDIENCE_SIDE_UP;

        return HiveState.UNKNOWN;

    }


    /**
     * Calculate the current HiveState for stability checks in update()
     * @return HiveState based on current AprilTagOrientation
     */
    private HiveState getPredictedState(AprilTagOrientation orientation, LLResultTypes.FiducialResult tag){
        if(orientation == getInitialOrientation(tag.getFiducialId())){
            return (isRedAlliance)? HiveState.AUDIENCE_SIDE_UP : HiveState.REAR_SIDE_UP;
        }else if( orientation == AprilTagOrientation.UNKNOWN){
            return HiveState.UNKNOWN;
        }
        return (isRedAlliance) ? HiveState.REAR_SIDE_UP : HiveState.AUDIENCE_SIDE_UP;
    }

    /**
     * @return AprilTagOrientation based on y-axis
     * @note A tag is "upright" when the Y axis points down
     */
    private AprilTagOrientation getTagOrientation(double yAxisVertical){
        if(yAxisVertical <= upThreshold){
            return AprilTagOrientation.UP;
        }else if(yAxisVertical >= downThreshold){
            return AprilTagOrientation.DOWN;
        }
        return AprilTagOrientation.UNKNOWN;
    }

    private double getYVertical(Pose3D pose){
        double roll = pose.getOrientation().getRoll(AngleUnit.RADIANS);
        double pitch = pose.getOrientation().getPitch(AngleUnit.RADIANS);
        double yaw = pose.getOrientation().getYaw(AngleUnit.RADIANS);

        double yY = Math.sin(yaw) * Math.sin(pitch) * Math.sin(roll) + Math.cos(yaw) * Math.cos(roll);
        double yZ = Math.cos(pitch) * Math.sin(roll);
        double cameraTilt = Math.toRadians(vision.tiltState.degrees);

        return -Math.cos(cameraTilt) * yY + Math.sin(cameraTilt) * yZ;
    }


    public HiveState getHiveState(){
      return hiveState;
    }

    public boolean isStale(){
      return isStale;
    }

    /**
     * Check what orientation a tag is initially in
     *
     * @return initial tag orientation
     */
    private AprilTagOrientation getInitialOrientation( int id){
      return (id >= 34 && id <= 37 || id >= 42 && id <= 45) ? AprilTagOrientation.UP : AprilTagOrientation.DOWN;
    }

    /**
     * @return true if AprilTag belongs to the blue hive, false otherwise  
     */
    public static boolean isBlueTag(int id){
        return  id >= 38 && id <= 45; 
    }

   /**
    * @return true if AprilTag belongs to the red hive, false otherwise  
    */
    public static boolean isRedTag(int id){
        return  id >= 30 && id <= 37;
    }
    
}
