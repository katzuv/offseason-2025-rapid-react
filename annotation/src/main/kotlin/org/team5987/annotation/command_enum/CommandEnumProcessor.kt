package org.team5987.annotation.command_enum

import com.google.devtools.ksp.processing.*
import com.google.devtools.ksp.symbol.*
import com.squareup.kotlinpoet.FileSpec
import com.squareup.kotlinpoet.FunSpec
import com.squareup.kotlinpoet.KModifier
import com.squareup.kotlinpoet.TypeSpec
import com.squareup.kotlinpoet.ClassName
import com.squareup.kotlinpoet.ksp.writeTo

const val ANNOTATION_PACKAGE = "org.team5987.annotation.command_enum.CommandEnum"

class CreateCommandProcessor(
    env: SymbolProcessorEnvironment
) : SymbolProcessor {
    private val code = env.codeGenerator

    override fun process(resolver: Resolver): List<KSAnnotated> {
        val symbols = resolver.getSymbolsWithAnnotation(ANNOTATION_PACKAGE)

        symbols
            .filterIsInstance<KSClassDeclaration>()
            .filter { it.classKind == ClassKind.ENUM_CLASS }
            .forEach { enumDecl ->
                generateForEnum(enumDecl)
            }

        return emptyList()
    }

    fun generateInterface(
        pkg: String,
        fileName: String,
        enumName: String,
        entries: List<String>
    ): FileSpec {
        val enumClass = ClassName(pkg, enumName)
        val commandClass = ClassName("edu.wpi.first.wpilibj2.command", "Command")

        // generate all default entry functions
        val entryFunctions = entries.map { entry ->
            FunSpec.builder(entry.lowercase())
                .returns(commandClass)
                .addStatement("return setAngle(%T.%L)", enumClass, entry)
                .build()
        }

        // abstract setAngle function
        val setAngleFun = FunSpec.builder("setAngle")
            .addParameter("value", enumClass)
            .returns(commandClass)
            .addModifiers(KModifier.ABSTRACT)
            .build()

        // the interface
        val interfaceSpec = TypeSpec.interfaceBuilder(fileName)
            .addFunctions(entryFunctions)
            .addFunction(setAngleFun)
            .build()

        // final file
        return FileSpec.builder(pkg, fileName)
            .addType(interfaceSpec)
            .build()
    }


    private fun generateForEnum(enumDecl: KSClassDeclaration) {
        val pkg = enumDecl.packageName.asString()
        val enumName = enumDecl.simpleName.asString()

        val entries = enumDecl.declarations
            .filterIsInstance<KSClassDeclaration>()
            .filter { it.classKind == ClassKind.ENUM_ENTRY }
            .map { it.simpleName.asString() }.toList()

        val fileName = "${enumName}CommandFactory"

        val generated: FileSpec = generateInterface(pkg, fileName, enumName, entries)

        generated.writeTo(code, Dependencies(false))
    }
}

class CommandEnumProcessorProvider : SymbolProcessorProvider {
    override fun create(environment: SymbolProcessorEnvironment): SymbolProcessor {
        return CreateCommandProcessor(environment)
    }
}