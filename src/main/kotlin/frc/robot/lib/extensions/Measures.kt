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

operator fun Distance.div(divisor: Number): Distance = this / divisor.toDouble()

operator fun Voltage.div(timeUnit: TimeUnit): Velocity<VoltageUnit> = this / timeUnit.one()

operator fun VoltageUnit.div(time: TimeUnit): VelocityUnit<VoltageUnit> = this / time

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

operator fun Distance.get(unit: DistanceUnit): Double = this.`in`(unit)

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

fun Angle.toRotation2d(): Rotation2d = Rotation2d(this[rad])

operator fun Angle.get(unit: AngleUnit): Double = this.`in`(unit)

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

// Other
val Number.sec: Time
    get() = toUnit(Units.Seconds::of)
val Number.seconds: Time
    get() = toUnit(Units.Seconds::of)

operator fun Time.get(unit: TimeUnit): Double = this.`in`(unit)

val Number.percent: Dimensionless
    get() = toUnit(Units.Percent::of)

operator fun Dimensionless.get(unit: DimensionlessUnit): Double = this.`in`(unit)

val Number.amps: Current
    get() = toUnit(Units.Amps::of)

operator fun Current.get(unit: CurrentUnit): Double = this.`in`(unit)

operator fun Voltage.get(unit: VoltageUnit): Double = this.`in`(unit)

operator fun Velocity<VoltageUnit>.get(unit: VelocityUnit<VoltageUnit>): Double = this.`in`(unit)

operator fun MomentOfInertia.get(unit: MomentOfInertiaUnit): Double = this.`in`(unit)

val Number.volts: Voltage
    get() = toUnit(Units.Volts::of)

val Number.kg2m: MomentOfInertia
    get() = toUnit(Units.KilogramSquareMeters::of)

val Number.kilogramSquareMeters: MomentOfInertia
    get() = toUnit(Units.KilogramSquareMeters::of)
