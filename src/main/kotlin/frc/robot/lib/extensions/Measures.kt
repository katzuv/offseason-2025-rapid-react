@file:Suppress("unused")

package frc.robot.lib.extensions

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.*
import edu.wpi.first.units.Unit as WpilibUnit
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.measure.*
import kotlin.math.PI

/**
 * # Unit DSL Extensions for WPILib
 *
 * Provides convenient Kotlin extensions for working with WPILib units using
 * readable syntax.
 *
 * ## Example Usage
 *
 * ```kotlin
 * // Distances
 * val d1 = 5.m
 * val d2 = 30.cm
 * val total = d1 + d2
 * val dInMm = d1[mm]
 *
 * // Angles
 * val a = 90.deg
 * val angleInRads = a[rad]
 * val rot2d = a.toRotation2d()
 *
 * // Velocities
 * val linear = 2.mps
 * val angular = linear.toAngular(diameter = 0.1.m, gearRatio = 3.0)
 *
 * // Time and Others
 * val t = 2.sec
 * val voltage = 6.volts
 * val current = 20.amps
 *
 * // Derived Units
 * val speed = total / t
 * ```
 *
 * These extensions allow for natural and concise code in robot control logic.
 */

// Length
val m: DistanceUnit = Units.Meters
val cm: DistanceUnit = Units.Centimeters
val mm: DistanceUnit = Units.Millimeters

// Angle
val deg: AngleUnit = Units.Degrees
val rad: AngleUnit = Units.Radians
val rot: AngleUnit = Units.Rotations

// Time
val sec: TimeUnit = Units.Seconds
val seconds: TimeUnit = Units.Seconds

// Other
val percent: DimensionlessUnit = Units.Percent
val amps: CurrentUnit = Units.Amps
val volts: VoltageUnit = Units.Volts
val kg2m: MomentOfInertiaUnit = Units.KilogramSquareMeters

val rps: AngularVelocityUnit
    get() = Units.RotationsPerSecond

val rps_squared: AngularAccelerationUnit
    get() = Units.RotationsPerSecond.per(sec)

val rps_tripled: VelocityUnit<AngularAccelerationUnit>
    get() = Units.RotationsPerSecondPerSecond.per(sec)

val rad_ps: AngularVelocityUnit
    get() = Units.RadiansPerSecond
val rad_ps_ps: AngularAccelerationUnit
    get() = Units.RadiansPerSecondPerSecond
val mps: LinearVelocityUnit
    get() = Units.MetersPerSecond

val mps_ps: LinearAccelerationUnit
    get() = Units.MetersPerSecondPerSecond
val deg_ps_ps: AngularAccelerationUnit
    get() = Units.DegreesPerSecondPerSecond

fun LinearVelocity.toAngular(
    diameter: Distance,
    gearRatio: Double,
): AngularVelocity =
    divideRatio(mps.per(rps).of(diameter[m] * gearRatio * PI))
        as AngularVelocity

fun Distance.toAngle(diameter: Distance, gearRatio: Double): Angle =
    divideRatio(m.per(rot).of(diameter[m] * gearRatio * PI)) as Angle

fun Angle.toDistance(diameter: Distance, gearRatio: Double): Distance =
    timesConversionFactor(m.per(rot).of(diameter[m] * gearRatio * PI))

fun AngularVelocity.toLinear(
    diameter: Distance,
    gearRatio: Double,
): LinearVelocity =
    timesConversionFactor(mps.per(rps).of(diameter[m] * gearRatio * PI))

operator fun Distance.div(time: TimeUnit): LinearVelocity = this / time.one()

operator fun <U : WpilibUnit> Measure<U>.div(divisor: Number): Measure<U> =
    this / divisor.toDouble()

@Suppress("UNCHECKED_CAST")
operator fun <T : WpilibUnit> T.div(timeUnit: TimeUnit): VelocityUnit<T> =
    this.per(timeUnit) as VelocityUnit<T>

@Suppress("UNCHECKED_CAST")
operator fun <T : WpilibUnit> Measure<T>.div(timeUnit: TimeUnit): Velocity<T> =
    (this / timeUnit.one()) as Velocity<T>

operator fun <U : WpilibUnit> Measure<U>.get(unit: U) = this.`in`(unit)

// Factories

// Helper function for conversion
inline fun <N : Number, R> N.toUnit(converter: (Double) -> R) =
    converter(toDouble())

// Distance
val Number.m: Distance
    get() = toUnit(Units.Meters::of)
val Number.cm: Distance
    get() = toUnit(Units.Centimeters::of)
val Number.mm: Distance
    get() = toUnit(Units.Millimeters::of)

// Linear velocity
val Number.mps: LinearVelocity
    get() = toUnit(Units.MetersPerSecond::of)

// Angle
val Number.deg: Angle
    get() = toUnit(Units.Degrees::of)
val Number.rot: Angle
    get() = toUnit(Units.Rotations::of)
val Number.rad: Angle
    get() = toUnit(Units.Radians::of)

fun Angle.toRotation2d(): Rotation2d = Rotation2d(`in`(Units.Radians))

// Angular velocity
val Number.deg_ps: AngularVelocity
    get() = toUnit(Units.DegreesPerSecond::of)
val Number.rot_ps: AngularVelocity
    get() = toUnit(Units.RotationsPerSecond::of)
val Number.rps: AngularVelocity
    get() = toUnit(Units.RotationsPerSecond::of)
val Number.rad_ps: AngularVelocity
    get() = toUnit(Units.RadiansPerSecond::of)

// Linear acceleration
val Number.mps_ps: LinearAcceleration
    get() = toUnit(Units.MetersPerSecondPerSecond::of)

// Angular acceleration and jerk
val Number.deg_ps_ps: AngularAcceleration
    get() = toUnit(Units.DegreesPerSecondPerSecond::of)

val Number.rps_squared: AngularAcceleration
    get() = this.rps / 1.sec

typealias AngularJerk = Velocity<AngularAccelerationUnit>

val Number.rps_tripled: AngularJerk
    get() = this.rps_squared / Second

// Other
val Number.sec: Time
    get() = toUnit(Units.Seconds::of)
val Number.seconds: Time
    get() = toUnit(Units.Seconds::of)

val Number.percent: Dimensionless
    get() = toUnit(Units.Percent::of)

val Number.amps: Current
    get() = toUnit(Units.Amps::of)

val Number.volts: Voltage
    get() = toUnit(Units.Volts::of)

val Number.kg2m: MomentOfInertia
    get() = toUnit(Units.KilogramSquareMeters::of)
