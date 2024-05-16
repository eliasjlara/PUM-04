package com.example.aida.socketcommunication

import org.junit.Assert.assertEquals
import org.junit.Test
class InstructionsTest{
    @Test
    fun testInstructionOn(){
        val expected = 1.toShort()
        assertEquals(expected, Instructions.ON.value)
    }
    @Test
    fun testInstructionOff(){
        val expected = 2.toShort()
        assertEquals(expected, Instructions.OFF.value)
    }
}