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
    

    public HiveTracker(boolean isRedAlliance, Vision vision){
        hiveState = (isRedAlliance) ?  HiveState.AUDIENCE_SIDE_UP: HiveState.REAR_SIDE_UP;
        candidateState = HiveState.UNKNOWN;
        this.vision = vision;
        this.isRedAlliance = isRedAlliance;
    }
    

    // ==================== UPDATE HIVE POSITION ====================

    /*
     * Updates hive position based on a fresh frame. 
     * @note Currently uses only the first valid FiducialResult. Will require testing   
     */
    public void update( List<LLResultTypes.FiducialResult> results ){
        if(results.isEmpty()) return;

        LLResultTypes.FiducialResult tag = results.get(0);
        Pose3D cameraToTag = tag.getTargetPoseCameraSpace();

        long timeNow_ms = System.nanoTime() / 1_000_000;

        if(hiveState == candidateState && timeNow_ms - startTime_ms >= timeThreshold_ms){

            double rx = tag.getTargetPoseCameraSpace().getOrientation().getRoll(AngleUnit.RADIANS);
            double ry = tag.getTargetPoseCameraSpace().getOrientation().getPitch(AngleUnit.RADIANS);
            double rz = tag.getTargetPoseCameraSpace().getOrientation().getYaw(AngleUnit.RADIANS);

            double yY = Math.sin(rz) * Math.sin(ry) * Math.sin(rx) + Math.cos(rz) * Math.cos(rx);
            double yZ = Math.cos(ry) * Math.sin(rx);

            AprilTagOrientation orientation = getOrientation(yY, yZ);

            HiveState observedState = getObservedState( orientation, tag);

            if(observedState != candidateState){
                // We have a new candiate state - restart the timer
                candidateState = observedState;
                startTime_ms = timeNow_ms;
            } else {
                // candidate state is stable - set hiveState to candidate
                hiveState = candidateState;
            }
        }else{
          // If hiveState is not stable, change state to UNKNOWN
          candidateState = hiveState;
          hiveState = HiveState.UNKNOWN;
        }
        

    }

    public HiveState getObservedState(AprilTagOrientation orientation, LLResultTypes.FiducialResult tag){
        HiveState observedState = null;
        if(orientation == getInitialOrientation(tag.getFiducialId())){
            if(isRedAlliance) observedState = HiveState.AUDIENCE_SIDE_UP;
            else observedState = HiveState.REAR_SIDE_UP;
        }else{
            if(isRedAlliance) observedState = HiveState.REAR_SIDE_UP;
            else observedState = HiveState.AUDIENCE_SIDE_UP;
        }
        return observedState;
    }


    public AprilTagOrientation getInitialOrientation( int id){
        return (id >= 34 && id <= 37 || id >= 42 && id <= 45) ? AprilTagOrientation.UP : AprilTagOrientation.DOWN;
    }


    public AprilTagOrientation getOrientation(double yY, double yZ){
        if(-Math.cos(Math.toRadians(vision.tiltState.degrees)) * yY + Math.sin(Math.toRadians(vision.tiltState.degrees)) * yZ <= upThreshold){
            return AprilTagOrientation.UP;
        }else if(-Math.cos(Math.toRadians(vision.tiltState.degrees)) * yY + Math.sin(Math.toRadians(vision.tiltState.degrees)) * yZ >= downThreshold){
            return AprilTagOrientation.DOWN;
        }

        return AprilTagOrientation.UNKNOWN;
    }


    // ==================== APRILTAG CHECKS   ====================

    /**
     * Check if an AprilTag can be used for blue alliance autoalign 
     *
     * @return true if AprilTag belongs to the blue hive, false otherwise  
     */
    public static boolean isBlueTag(int id){
        return  id >= 38 && id <= 45; 
    }

   /**
    * Check if an AprilTag can be used for red alliance autoalign 
    *
    * @return true if AprilTag belongs to the red hive, false otherwise  
    */
    public static boolean isRedTag(int id){
        return  id >= 30 && id <= 37;
    }
    
}
