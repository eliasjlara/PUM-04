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
        checkIfJoystickDataValid(x, y)
        val id = MessageType.JOYSTICK.value
        // The values are floats of size 4 bytes
        // Therefore the message is 8 bytes long with 2 floats
        val size = 8
        val buffer = ByteBuffer.allocate(size)
        buffer.putFloat(x)
        buffer.putFloat(y)
        sendDataToServer(id, buffer.array())
    }

    /**
     * Check if the joystick data to be sent is valid
     * @param x The x value of the joystick - float value between -1 and 1
     * @param y The y value of the joystick - float value between -1 and 1
     * @throws IllegalArgumentException if the joystick values are not between -1 and 1
     */
    fun checkIfJoystickDataValid(x : Float, y : Float){
        if(x < -1 || x > 1 || y < -1 || y > 1){
            throw IllegalArgumentException("Joystick values must be between -1 and 1")
        }
    }

    // TODO - This function may not be needed and can be removed?
    fun sendStartJoystick(){
        val id = MessageType.JOYSTICK.value
        val size = 2
        val start = 1
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start.toShort())
        sendDataToServer(id, buffer.array())
    }
}