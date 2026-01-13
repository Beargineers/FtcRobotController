package org.beargineers.platform.decode

import org.beargineers.platform.Alliance
import org.beargineers.platform.Angle
import org.beargineers.platform.Distance
import org.beargineers.platform.Location
import org.beargineers.platform.Robot
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

    val shootingAngleCorrection: Angle

    val locations : Locations get() = Locations(this)
}

class Locations(val robot: DecodeRobot) {
    val GOAL get() = if (robot.opMode.alliance == Alliance.RED) RED_GOAL else BLUE_GOAL
    val PARK get() = if (robot.opMode.alliance == Alliance.RED) RED_PARK else BLUE_PARK
    val OPEN_RAMP get() = if (robot.opMode.alliance == Alliance.RED) RED_OPEN_RAMP else BLUE_OPEN_RAMP
    val OPEN_RAMP_APPROACH get() = if (robot.opMode.alliance == Alliance.RED) RED_OPEN_RAMP_APPROACH else BLUE_OPEN_RAMP_APPROACH

    val OPEN_RAMP_SPEED by robot.config(0.6)

    private val RED_GOAL by robot.config(tileLocation("F6TR"))
    private val BLUE_GOAL by robot.config(tileLocation("A6TL"))

    private val RED_PARK by robot.config(tileLocation("B2BR").shift(-9.inch, -9.inch))
    private val BLUE_PARK by robot.config(tileLocation("E2BL").shift(-9.inch, 9.inch))

    private val RED_OPEN_RAMP by robot.config(-6.cm, 129.cm, 90.degrees)
    private val RED_OPEN_RAMP_APPROACH by robot.config(-6.cm, 80.cm, 90.degrees)

    private val BLUE_OPEN_RAMP by robot.config(-6.cm, -129.cm, -90.degrees)
    private val BLUE_OPEN_RAMP_APPROACH by robot.config(-6.cm, -80.cm, -90.degrees)
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

    fun pointInShootingZone(p: Location): Boolean{
        if (shootingZone == ShootingZones.CLOSEST) {
            return if (p.x > 0.inch) { // far shooting zones
                if (p.y > 0.inch) { // red far shooting zone
                    p.y < p.x - 48.inch
                } else { // Blue far shooting zone
                    -p.y < p.x - 48.inch
                }
            } else { // close shooting zone
                if (p.y > 0.inch) {// red
                    p.y < -p.x
                } else { // blue
                    -p.y < -p.x
                }
            }
        }else if (shootingZone == ShootingZones.FRONT){// close shooting zone
            return  if (p.y > 0.inch) {// red
                p.y < -p.x
            } else { // blue
                -p.y < -p.x

            }
        }else{ // far shooting zone
            return if (p.y > 0.inch) { // red far shooting zone
                p.y < p.x - 48.inch
            } else { // Blue far shooting zone
                -p.y < p.x - 48.inch
            }
        }
    }

    return pointInShootingZone(rf_wheel) ||
            pointInShootingZone(lf_wheel) ||
            pointInShootingZone(rb_wheel) ||
            pointInShootingZone(lb_wheel)
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
    val alliance = opMode.alliance
    val far = if (alliance == Alliance.BLUE){
        Location(55.inch, (-12).inch) // BLUE
    }
    else {
        Location(60.inch, 12.inch) // RED
    }

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