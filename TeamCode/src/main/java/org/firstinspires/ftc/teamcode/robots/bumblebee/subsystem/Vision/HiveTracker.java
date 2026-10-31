package org.firstinspires.ftc.teamcode.robots.bumblebee.subsystem.Vision;

import java.util.List;

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
    public static double upThreshold = -0.2;
    public static double downThreshold = 0.2;
    
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

    /**
     * Updates hive position based on a fresh frame. 
     * @note Currently uses only the first valid FiducialResult. Will require testing   
     */
    public void update( List<LLResultTypes.FiducialResult> results ){
        long timeNow_ms = System.nanoTime() / 1_000_000;

        // Set candidate to UNKNOWN and restart timer  - no state is visible
        if(results.isEmpty()){
            startTime_ms = timeNow_ms;
            candidateState = HiveState.UNKNOWN;
            return;
        }

        LLResultTypes.FiducialResult tag = results.get(0);

        double rx = tag.getTargetPoseCameraSpace().getOrientation().getRoll(AngleUnit.RADIANS);
        double ry = tag.getTargetPoseCameraSpace().getOrientation().getPitch(AngleUnit.RADIANS);
        double rz = tag.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.RADIANS);

        double yY = Math.sin(rz) * Math.sin(ry) * Math.sin(rx) + Math.cos(rz) * Math.cos(rx);
        double yZ = Math.cos(ry) * Math.sin(rx);

        AprilTagOrientation orientation = getOrientation(yY, yZ);
        HiveState observedState = getObservedState( orientation, tag);

        // If the current observedState does not match the candidate
        if(observedState != candidateState){
            // We have a new candiate state - restart the timer
            candidateState = observedState;
            startTime_ms = timeNow_ms;
        }  
        // If we have passed the timeThreshold_ms, candidateState is confirmed stable
        if(timeNow_ms - startTime_ms >= timeThreshold_ms){
            // candidate state is stable - set hiveState to candidate
            hiveState = candidateState;
        }else{
            hiveState = HiveState.UNKNOWN;
        }


    }

    // ==================== APRILTAG CHECKS   ====================
    
    /**
     * Apply transform considering camera tilt to determine direction of the tags Y axis 
     *
     * @note A tag is "upright" when the Y axis points down 
     * @return AprilTagOrientation based on y-axis
     */
    private AprilTagOrientation getOrientation(double yY, double yZ){
      if(-Math.cos(Math.toRadians(vision.tiltState.degrees)) * yY + Math.sin(Math.toRadians(vision.tiltState.degrees)) * yZ <= upThreshold){
        return AprilTagOrientation.UP;
      }else if(-Math.cos(Math.toRadians(vision.tiltState.degrees)) * yY + Math.sin(Math.toRadians(vision.tiltState.degrees)) * yZ >= downThreshold){
        return AprilTagOrientation.DOWN;
      }

      return AprilTagOrientation.UNKNOWN;
    }

    
    /**
     * Calculate the current HiveState for stability checks in update()
     * @return HiveState based on current AprilTagOrientation   
     */
    private HiveState getObservedState(AprilTagOrientation orientation, LLResultTypes.FiducialResult tag){
      if(orientation == getInitialOrientation(tag.getFiducialId())){
        return (isRedAlliance)? HiveState.AUDIENCE_SIDE_UP : HiveState.REAR_SIDE_UP;
      }else if( orientation == AprilTagOrientation.UNKNOWN){
        return HiveState.UNKNOWN;
      }
      return (isRedAlliance) ? HiveState.REAR_SIDE_UP : HiveState.AUDIENCE_SIDE_UP;
    }


    public HiveState getHiveState(){
      return hiveState;
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
