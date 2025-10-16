@file:Suppress("unused")

package frc.robot.lib.extensions

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.*
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
val meters: DistanceUnit = Units.Meters
val cm: DistanceUnit = Units.Centimeters
val centimeters: DistanceUnit = Units.Centimeters
val mm: DistanceUnit = Units.Millimeters
val millimeters: DistanceUnit = Units.Millimeters

// Angle
val deg: AngleUnit = Units.Degrees
val degrees: AngleUnit = Units.Degrees
val rad: AngleUnit = Units.Radians
val radians: AngleUnit = Units.Radians
val rot: AngleUnit = Units.Rotations
val rotations: AngleUnit = Units.Rotations

// Time
val sec: TimeUnit = Units.Seconds
val seconds: TimeUnit = Units.Seconds

// Other
val percent: DimensionlessUnit = Units.Percent
val amps: CurrentUnit = Units.Amps
val volts: VoltageUnit = Units.Volts
val kg2m: MomentOfInertiaUnit = Units.KilogramSquareMeters

val rps_squared
    get() = Units.RotationsPerSecond.per(sec)

val rad_ps
    get() = Units.RadiansPerSecond
val rad_ps_ps
    get() = Units.RadiansPerSecondPerSecond
val mps
    get() = Units.MetersPerSecond

val mps_ps
    get() = Units.MetersPerSecondPerSecond
val deg_ps_ps
    get() = Units.DegreesPerSecondPerSecond

fun LinearVelocity.toAngular(
    diameter: Distance,
    gearRatio: Double,
): AngularVelocity = timesConversionFactor(
    Units.RotationsPerSecond.per(Units.MetersPerSecond).of(1.0 / (diameter[m] * gearRatio * PI))
)

fun Distance.toAngle(diameter: Distance, gearRatio: Double): Angle = timesConversionFactor(
    Units.Rotations.per(Units.Meters).of(1.0 / (diameter[m] * gearRatio * PI))
)

fun Angle.toDistance(diameter: Distance, gearRatio: Double): Distance = timesConversionFactor(
    Units.Meters.per(Units.Rotations).of(diameter[m] * gearRatio * PI)
)

fun AngularVelocity.toLinear(
    diameter: Distance,
    gearRatio: Double,
): LinearVelocity = timesConversionFactor(
    Units.MetersPerSecond.per(Units.RotationsPerSecond).of(diameter[m] * gearRatio * PI)
)

operator fun Distance.div(time: TimeUnit): LinearVelocity = this / time.one()

@Suppress("RemoveRedundantQualifierName") // Emphasizing this is not Kotlin's [Unit].
operator fun <U : edu.wpi.first.units.Unit> Measure<U>.div(divisor: Number): Measure<U> = this / divisor.toDouble()

operator fun VoltageUnit.div(timeUnit: TimeUnit): VelocityUnit<VoltageUnit> = this.per(timeUnit)

operator fun Voltage.div(timeUnit: TimeUnit): Velocity<VoltageUnit> = this / timeUnit.one()

@Suppress("RemoveRedundantQualifierName") // Emphasizing this is not Kotlin's [Unit].
operator fun <U : edu.wpi.first.units.Unit> Measure<U>.get(unit: U) = this.`in`(unit)

// Factories

// Helper function for conversion
inline fun <N : Number, R> N.toUnit(converter: (Double) -> R) = converter(toDouble())

// Distance
val Number.m: Distance
    get() = toUnit(Units.Meters::of)
val Number.meters: Distance
    get() = toUnit(Units.Meters::of)
val Number.cm: Distance
    get() = toUnit(Units.Centimeters::of)
val Number.centimeters: Distance
    get() = toUnit(Units.Centimeters::of)
val Number.mm: Distance
    get() = toUnit(Units.Millimeters::of)
val Number.millimeters: Distance
    get() = toUnit(Units.Millimeters::of)

// Linear velocity
val Number.mps: LinearVelocity
    get() = toUnit(Units.MetersPerSecond::of)

// Angle
val Number.deg: Angle
    get() = toUnit(Units.Degrees::of)
val Number.degrees: Angle
    get() = toUnit(Units.Degrees::of)
val Number.rot: Angle
    get() = toUnit(Units.Rotations::of)
val Number.rotations: Angle
    get() = toUnit(Units.Rotations::of)
val Number.rad: Angle
    get() = toUnit(Units.Radians::of)
val Number.radians: Angle
    get() = toUnit(Units.Radians::of)

fun Angle.toRotation2d(): Rotation2d = Rotation2d(`in`(Units.Radians))

// Angular velocity
val Number.deg_ps: AngularVelocity
    get() = toUnit(Units.DegreesPerSecond::of)
val Number.degreesPerSecond: AngularVelocity
    get() = toUnit(Units.DegreesPerSecond::of)
val Number.rot_ps: AngularVelocity
    get() = toUnit(Units.RotationsPerSecond::of)
val Number.rotationsPerSecond: AngularVelocity
    get() = toUnit(Units.RotationsPerSecond::of)
val Number.rps: AngularVelocity
    get() = toUnit(Units.RotationsPerSecond::of)
val Number.rad_ps: AngularVelocity
    get() = toUnit(Units.RadiansPerSecond::of)
val Number.radiansPerSecond: AngularVelocity
    get() = toUnit(Units.RadiansPerSecond::of)

// Linear acceleration
val Number.mps_ps: LinearAcceleration
    get() = toUnit(Units.MetersPerSecondPerSecond::of)

// Angular acceleration
val Number.deg_ps_ps: AngularAcceleration
    get() = toUnit(Units.DegreesPerSecondPerSecond::of)

//
val Number.rps_squared: AngularAcceleration
    get() = toUnit(Units.RotationsPerSecond::of).per(Units.Second)

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

val Number.kilogramSquareMeters: MomentOfInertia
    get() = toUnit(Units.KilogramSquareMeters::of)
