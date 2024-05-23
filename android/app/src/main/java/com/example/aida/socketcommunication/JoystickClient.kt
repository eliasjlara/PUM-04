package com.example.aida.socketcommunication

import java.nio.ByteBuffer

/**
 * Class is created to send joystick data to the server.
 * The joystick data is sent as a float value between -1 and 1
 */
class JoystickClient(ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) : AbstractClient(ip, port, timeToTimeout){

    /**
     * Send joystick data to server
     * @param x The x value of the joystick - float value between -1 and 1
     * @param y The y value of the joystick - float value between -1 and 1
     */
    fun sendJoystickData(x : Float, y : Float){
        try {
            checkIfJoystickDataValid(x, y)
        } catch (e : IllegalArgumentException){
            println(e.message)
            throw IllegalArgumentException("Joystick values must be between -1 and 1")
        }
        val id = MessageType.JOYSTICK.value
        // The values are floats of size 4 bytes
        // Therefore the message is 8 bytes long with 2 floats
        val size = 8
        val buffer = ByteBuffer.allocate(size)
        buffer.putFloat(x)
        buffer.putFloat(y)
        println("Sending joystick data to server: x: $x, y: $y")
        sendDataToServer(id, buffer.array())
    }

    /**
     * Check if the joystick data to be sent is valid
     * @param x The x value of the joystick - float value between -1 and 1
     * @param y The y value of the joystick - float value between -1 and 1
     * @throws IllegalArgumentException if the joystick values are not between -1 and 1
     */
    private fun checkIfJoystickDataValid(x : Float, y : Float){
        if(x < -1 || x > 1 || y < -1 || y > 1){
            throw IllegalArgumentException("Joystick values must be between -1 and 1")
        }
    }
}