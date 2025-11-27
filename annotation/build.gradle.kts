plugins {
    kotlin("jvm")
    id("edu.wpi.first.GradleRIO") version "2025.3.2"
}

group = "org.team5987"

dependencies {
    implementation("com.squareup:kotlinpoet:1.14.2")
    implementation("com.squareup:kotlinpoet-ksp:1.14.2")
    implementation("com.google.devtools.ksp:symbol-processing-api:2.0.0-1.0.21")

    implementation("edu.wpi.first.wpilibj:wpilibj-java:2025.3.2")
    implementation("edu.wpi.first.wpilibNewCommands:wpilibNewCommands-java:2025.3.2")

}

repositories {
    mavenCentral()
}

sourceSets.main {
    java.srcDirs("src/main/kotlin")
}

