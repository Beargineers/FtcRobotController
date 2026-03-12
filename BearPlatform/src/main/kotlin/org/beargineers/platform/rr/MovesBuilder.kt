package org.beargineers.platform.rr

import com.acmerobotics.roadrunner.AccelConstraint
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.TrajectoryBuilder
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.VelConstraint
import org.beargineers.platform.Angle
import org.beargineers.platform.Distance
import org.beargineers.platform.Location
import org.beargineers.platform.Position

class MovesBuilder(private var rrBuilder: TrajectoryBuilder) {
    var curVelocityContsraint: VelConstraint? = null
    var curAccelerationConstraint: AccelConstraint? = null

    fun setTangent(r: Angle) {
        rrBuilder = rrBuilder.setTangent(r.radians())
    }

    fun setReversed(reversed: Boolean) {
        rrBuilder = rrBuilder.setReversed(reversed)
    }

    fun constrainVelocity(velConstraint: VelConstraint, body: () -> Unit) {
        val old = curVelocityContsraint
        body()
        curVelocityContsraint = old
    }

    fun constrainAcceleration(accelConstraint: AccelConstraint, body: () -> Unit) {
        val old = curAccelerationConstraint
        body()
        curAccelerationConstraint = old
    }

    fun lineToX(posX: Distance) {
        rrBuilder = rrBuilder.lineToX(posX.inch(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun lineToXConstantHeading(posX: Distance) {
        rrBuilder = rrBuilder.lineToXConstantHeading(posX.inch(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun lineToXLinearHeading(posX: Distance, heading: Angle) {
        rrBuilder = rrBuilder.lineToXLinearHeading(posX.inch(), heading.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun lineToXSplineHeading(posX: Distance, heading: Angle) {
        rrBuilder = rrBuilder.lineToXSplineHeading(posX.inch(), heading.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun lineToY(posY: Distance) {
        rrBuilder = rrBuilder.lineToY(posY.inch(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun lineToYConstantHeading(posY: Distance) {
        rrBuilder = rrBuilder.lineToYConstantHeading(posY.inch(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun lineToYLinearHeading(posY: Distance, heading: Angle) {
        rrBuilder = rrBuilder.lineToYLinearHeading(posY.inch(), heading.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun lineToYSplineHeading(posY: Distance, heading: Angle) {
        rrBuilder = rrBuilder.lineToYSplineHeading(posY.inch(), heading.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    private fun Location.toVector2D() = Vector2d(x.inch(), y.inch())
    private fun Position.toPose2D() = Pose2d(location().toVector2D(), heading.radians())

    fun strafeTo(loc: Location) {
        rrBuilder = rrBuilder.strafeTo(loc.toVector2D(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun strafeToConstantHeading(loc: Location) {
        rrBuilder = rrBuilder.strafeToConstantHeading(loc.toVector2D(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun strafeToLinearHeading(position: Position) {
        rrBuilder = rrBuilder.strafeToLinearHeading(position.location().toVector2D(), position.heading.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun strafeToSplineHeading(position: Position) {
        rrBuilder = rrBuilder.strafeToSplineHeading(position.location().toVector2D(), position.heading.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun splineTo(location: Location, tangent: Angle) {
        rrBuilder = rrBuilder.splineTo(location.toVector2D(), tangent.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun splineToConstantHeading(location: Location, tangent: Angle) {
        rrBuilder = rrBuilder.splineToConstantHeading(location.toVector2D(), tangent.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun splineToLinearHeading(position: Position, tangent: Angle) {
        rrBuilder = rrBuilder.splineToLinearHeading(position.toPose2D(), tangent.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun splineToSplineHeading(position: Position, tangent: Angle) {
        rrBuilder = rrBuilder.splineToSplineHeading(position.toPose2D(), tangent.radians(), curVelocityContsraint, curAccelerationConstraint)
    }

    fun build() = rrBuilder.build()
}