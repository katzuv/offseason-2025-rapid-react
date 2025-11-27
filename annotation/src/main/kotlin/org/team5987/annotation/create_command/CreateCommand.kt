package org.team5987.annotation.create_command

import edu.wpi.first.wpilibj2.command.Command

@Target(AnnotationTarget.FUNCTION, AnnotationTarget.CLASS)
@Retention(AnnotationRetention.SOURCE)
annotation class CreateCommand()
