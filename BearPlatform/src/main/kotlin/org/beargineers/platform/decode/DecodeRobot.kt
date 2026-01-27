package org.beargineers.platform.decode

import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.Distance
import org.beargineers.platform.Location
import org.beargineers.platform.Position
import org.beargineers.platform.Robot
import org.beargineers.platform.RobotLocations
import org.beargineers.platform.abs
import org.beargineers.platform.atan2
import org.beargineers.platform.cm
import org.beargineers.platform.config
import org.beargineers.platform.degrees
import org.beargineers.platform.hypot
import org.beargineers.platform.inch
import org.beargineers.platform.tileLocation

interface DecodeRobot : Robot {
    val intakeMode: IntakeMode
    fun intakeMode(mode: IntakeMode)
    fun launch()
    fun enableFlywheel(on: Boolean)

    fun isShooting(): Boolean

    fun adjustShooting(distance: Double, angle: Double)

    val shootingAngleCorrection: Angle

    val locations : Locations get() = Locations(this)

    val artifactsCount: Int
}

class Locations(val robot: DecodeRobot) {
    val GOAL get() = RED_GOAL.mirrorForAlliance(robot)
    val PARK get() = RED_PARK.mirrorForAlliance(robot)

    val OPEN_RAMP get() = RED_OPEN_RAMP.mirrorForAlliance(robot)
    val OPEN_RAMP_APPROACH get() = RED_OPEN_RAMP_APPROACH.mirrorForAlliance(robot)
    val OPEN_RAMP_COLLECT get() = RED_OPEN_RAMP_COLLECT.mirrorForAlliance(robot)
    val OPEN_RAMP_COLLECT_APPROACH get() = RED_OPEN_RAMP_COLLECT_APPROACH.mirrorForAlliance(robot)

    val COLLECT_FROM_OPEN_RAMP get() = RED_COLLECT_FROM_OPEN_RAMP.mirrorForAlliance(robot)

    val OPEN_RAMP_SPEED by config(0.6)

    private val RED_GOAL by config(tileLocation("F6TR"))
    private val RED_PARK by config(tileLocation("B2BR").shift(-9.inch, -9.inch))
    private val RED_OPEN_RAMP_COLLECT by config(-6.cm, 129.cm, 90.degrees)
    private val RED_OPEN_RAMP_COLLECT_APPROACH by config(-6.cm, 80.cm, 90.degrees)

    private val RED_COLLECT_FROM_OPEN_RAMP by config(50.cm, 152.cm, 152.degrees)

    private val RED_OPEN_RAMP by config(-6.cm, 129.cm, 90.degrees)
    private val RED_OPEN_RAMP_APPROACH by config(-6.cm, 80.cm, 90.degrees)

    val SPIKE_APPROACH_Y by config(61.0)
    val SPIKE_FINAL_Y by config(143.0) // 20cm less for Spike#3

    val SPIKE_SCOOPING_SPEED by config(1.0)
    val INITIAL_SHOT_SPEED by config(1.0)
}

fun Position.mirrorForAlliance(robot: DecodeRobot): Position {
    return mirrorForAlliance(robot.alliance)
}

fun Position.mirrorForAlliance(alliance: Alliance): Position {
    return if (alliance == Alliance.RED) this else Position(x, -y, -heading)
}

fun Location.mirrorForAlliance(robot: DecodeRobot): Location {
    return if (robot.alliance == Alliance.RED) this else Location(x, -y)
}

fun DecodeRobot.goalDistance(): Distance {
    val goal = locations.GOAL
    val cp = currentPosition
    return hypot(cp.x - goal.x, cp.y - goal.y)
}

fun DecodeRobot.shootingAngleCorrectionForMovement() : Angle {
    return 0.degrees
}

fun DecodeRobot.headingToGoal(): Angle {
    return headingToGoalFrom(currentPosition.location())
}

fun DecodeRobot.headingToGoalFrom(position: Location): Angle {
    val goal = locations.GOAL
    val dx = goal.x - position.x
    val dy = goal.y - position.y
    return atan2(dy, dx) + shootingAngleCorrectionForMovement() + shootingAngleCorrection
}

enum class ShootingZones {
    CLOSEST, FRONT, BACK
}
fun DecodeRobot.inShootingZone(shootingZone: ShootingZones = ShootingZones.CLOSEST): Boolean{

    fun pointInCloseShootingZone(p: Location): Boolean{
       if (p.x.cm() < 0){
           return abs(p.x)>(abs(p.y))
       }else{
           return false
       }
    }

    fun pointInFarShootingZone(p: Location): Boolean {
        if (p.x > 48.inch) {
            return abs(p.x - 48.inch) > abs(p.y)
        } else {
            return false
        }
    }

    fun pointInShootingZone(p: Location): Boolean{
        if (shootingZone == ShootingZones.CLOSEST) {
            return (pointInCloseShootingZone(p) || pointInFarShootingZone(p))
        }else if (shootingZone == ShootingZones.FRONT){// close shooting zone
            return (pointInCloseShootingZone(p))
        }else{ // far shooting zone
            return (pointInFarShootingZone(p))
        }
    }

    val l = getPart(RobotLocations)

    return pointInShootingZone(l.rf_wheel) ||
            pointInShootingZone(l.lf_wheel) ||
            pointInShootingZone(l.rb_wheel) ||
            pointInShootingZone(l.lb_wheel)
}

fun DecodeRobot.clearForShooting(): Boolean{
    fun headingIsAtGoal(): Boolean{
        val sideDistanceDeviation = 15.cm
        val distanceToGoal = goalDistance()
        val maxHeadingDeviation = atan2(sideDistanceDeviation, distanceToGoal)
        return currentPosition.heading > headingToGoal() - maxHeadingDeviation && currentPosition.heading < headingToGoal() + maxHeadingDeviation
    }

    fun flySpeedIsCorrect(): Boolean{
        return true
    }
    // TODO: add a check if the speed if the flywheel is good
    return inShootingZone() && headingIsAtGoal() && flySpeedIsCorrect()
}


fun DecodeRobot.closestPointInShootingZone(shootingZone: ShootingZones): Location{
    if ((inShootingZone() && shootingZone == ShootingZones.CLOSEST) || (shootingZone == ShootingZones.FRONT && inShootingZone(ShootingZones.FRONT)) || (shootingZone == ShootingZones.BACK && inShootingZone(
            ShootingZones.BACK))){
        return currentPosition.location()
    }
    val far = AutoPositions.SOUTH_SHOOTING.mirrorForAlliance(this).location() // far
    val close = if (currentPosition.y <= 0.cm){ // blue side
        if (currentPosition.y < -currentPosition.x){
            Location((currentPosition.x + currentPosition.y)*0.5, (currentPosition.x + currentPosition.y)*0.5)
        } else {
            Location(0.cm, 0.cm)
        }
    } else { // RED side
        if (currentPosition.y > currentPosition.x) {
            Location((currentPosition.y - currentPosition.x)*0.5, -(currentPosition.y - currentPosition.x)*0.5)
        }else{
            Location(0.cm, 0.cm)
        }
    }
    return if (shootingZone == ShootingZones.CLOSEST){
        if (currentPosition.location().distanceTo(close) < currentPosition.location().distanceTo(far)){
            close
        } else {
            far
        }
    }else if(shootingZone == ShootingZones.FRONT){
        close
    }else{
        far
    }
}

fun DecodeRobot.spikeStart(n: Int): Position {
    val x = ((2 - n)*24).inch + 12.inch
    val y = locations.SPIKE_APPROACH_Y

    return Position(x, y.cm, 90.degrees).mirrorForAlliance(this)
}

fun DecodeRobot.spikeEnd(n: Int): Position {
    val x = ((2 - n)*24).inch + 12.inch
    val y = if (n == 3) (locations.SPIKE_FINAL_Y - 20) else locations.SPIKE_FINAL_Y

    return Position(x, y.cm, 90.degrees).mirrorForAlliance(this)
}

fun main() {
    println(tileLocation("F6TR"))
}