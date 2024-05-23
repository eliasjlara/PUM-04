package com.example.aida.socketcommunication

import java.nio.ByteBuffer

/**
 * This class sends instructions to the server to toggle different nodes on and off
 */
class InstructionClient(ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) : AbstractClient(ip, port, timeToTimeout) {
    fun sendGestureInstruction(instruction: Instructions){
        val id = MessageType.IMAGE_ANALYSIS.value
        val size = 2
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(instruction.value)
        sendDataToServer(id, buffer.array())
    }

    fun sendStopCamera() {
        val id = MessageType.CAMERA.value
        val size = 2
        val stop = Instructions.OFF.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(stop)
        sendDataToServer(id, buffer.array())
    }

    fun sendStartCamera(){
        val id = MessageType.CAMERA.value
        val size = 2
        val start = Instructions.ON.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start)
        sendDataToServer(id, buffer.array())
    }
}