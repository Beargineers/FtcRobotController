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
import org.beargineers.platform.between
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
    fun prepareForShooting()
    fun enableFlywheel(on: Boolean)

    fun isShooting(): Boolean

    fun adjustShooting(distance: Double, angle: Double)

    val shootingAngleCorrection: Angle

    val artifactsCount: Int

    val hasTurret: Boolean

    val optimalArtifactStrafe: Distance get() = 0.cm

    /**
     *  Angle at which an artifact would be fired if it happens now. Naturally, it is a heading of a turret
     *  if the robot has it or just a heading of a robot if it shoots forward.
     */
    val shooterAngle: Angle get() = currentPosition.heading
}

object Locations {
    val OPEN_RAMP_SPEED by config(0.6)

    val GOAL by config(tileLocation("F6TR"))
    val PARK by config(tileLocation("B2BR").shift(-9.inch, -9.inch))
    val OPEN_RAMP_COLLECT by config(-6.cm, 129.cm, 90.degrees)
    val OPEN_RAMP_COLLECT_APPROACH by config(-6.cm, 80.cm, 90.degrees)
    val COLLECT_FROM_OPEN_RAMP by config(50.cm, 152.cm, 152.degrees)

    val OPEN_RAMP by config(-6.cm, 129.cm, 90.degrees)
    val OPEN_RAMP_APPROACH by config(-6.cm, 80.cm, 90.degrees)

    val SPIKE_APPROACH_Y by config(61.0)
    val SPIKE_FINAL_Y by config(143.0) // 20cm less for Spike#3

    val SPIKE_SCOOPING_SPEED by config(1.0)
    val INITIAL_SHOT_SPEED by config(1.0)
}

fun Position.mirrorForAlliance(alliance: Alliance): Position {
    return if (alliance == Alliance.RED) this else Position(x, -y, -heading)
}

fun Location.mirrorForAlliance(alliance: Alliance): Location {
    return if (alliance == Alliance.RED) this else Location(x, -y)
}

fun DecodeRobot.goalDistance(): Distance {
    val goal = Locations.GOAL.mirrorForAlliance(alliance)
    val cp = currentPosition
    return hypot(cp.x - goal.x, cp.y - goal.y)
}

fun DecodeRobot.headingToGoal(): Angle {
    return headingToGoalFrom(currentPosition.location())
}

fun DecodeRobot.headingToGoalFrom(position: Location): Angle {
    val goal = Locations.GOAL.mirrorForAlliance(alliance)
    val dx = goal.x - position.x
    val dy = goal.y - position.y
    return atan2(dy, dx) + shootingAngleCorrection
}

enum class ShootingZones {
    CLOSEST, FRONT, BACK
}
fun DecodeRobot.inShootingZone(shootingZone: ShootingZones = ShootingZones.CLOSEST): Boolean {

    fun pointInCloseShootingZone(p: Location): Boolean {
        if (p.x.cm() < 0) {
            return abs(p.x) > (abs(p.y))
        } else {
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

    fun pointInShootingZone(p: Location): Boolean {
        if (shootingZone == ShootingZones.CLOSEST) {
            return (pointInCloseShootingZone(p) || pointInFarShootingZone(p))
        } else if (shootingZone == ShootingZones.FRONT) {// close shooting zone
            return (pointInCloseShootingZone(p))
        } else { // far shooting zone
            return (pointInFarShootingZone(p))
        }
    }

    val l = getPart(RobotLocations)

    val points = listOf(
        l.rf_corner,
        l.lf_corner,
        l.rb_corner,
        l.rb_corner,
        l.rf_corner.between(l.lf_corner),
        l.lf_corner.between(l.lb_corner),
        l.lb_corner.between(l.rb_corner),
        l.rb_corner.between(l.rf_corner)
    )

    return points.any { pointInShootingZone(it) }
}

fun DecodeRobot.clearForShooting(): Boolean{
    fun headingIsAtGoal(): Boolean{
        val sideDistanceDeviation = 15.cm
        val distanceToGoal = goalDistance()
        val maxHeadingDeviation = atan2(sideDistanceDeviation, distanceToGoal)
        val headingToGoal = headingToGoal()
        return shooterAngle in headingToGoal - maxHeadingDeviation .. headingToGoal + maxHeadingDeviation
    }

    fun flySpeedIsCorrect(): Boolean{
        return true
    }
    // TODO: add a check if the speed if the flywheel is good
    return inShootingZone() && headingIsAtGoal() && flySpeedIsCorrect()
}


fun DecodeRobot.closestPointInShootingZone(shootingZone: ShootingZones, position: Location = currentPosition.location()): Location{
    val cp = position
    if ((inShootingZone() && shootingZone == ShootingZones.CLOSEST) || (shootingZone == ShootingZones.FRONT && inShootingZone(ShootingZones.FRONT)) || (shootingZone == ShootingZones.BACK && inShootingZone(
            ShootingZones.BACK))){
        return cp
    }
    val far = AutoPositions.SOUTH_SHOOTING.mirrorForAlliance(alliance).location() // far
    val close = if (cp.y <= 0.cm){ // blue side
        if (cp.y < -cp.x){
            Location((cp.x + cp.y)*0.5, (cp.x + cp.y)*0.5)
        } else {
            Location(0.cm, 0.cm)
        }
    } else { // RED side
        if (cp.y > cp.x) {
            Location(-(cp.y - cp.x)*0.5, (cp.y - cp.x)*0.5)
        }else{
            Location(0.cm, 0.cm)
        }
    }
    return if (shootingZone == ShootingZones.CLOSEST){
        if (cp.distanceTo(close) < cp.distanceTo(far)){
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

fun spikeStart(n: Int): Position {
    val x = ((2 - n)*24).inch + 12.inch
    val y = Locations.SPIKE_APPROACH_Y

    return Position(x, y.cm, 90.degrees)
}

fun spikeEnd(n: Int): Position {
    val x = ((2 - n)*24).inch + 12.inch
    val y = if (n == 3) (Locations.SPIKE_FINAL_Y - 20) else Locations.SPIKE_FINAL_Y

    return Position(x, y.cm, 90.degrees)
}