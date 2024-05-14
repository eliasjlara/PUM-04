package com.example.aida.socketcommunication

import java.nio.ByteBuffer
// TODO - Add function that receives Lidar data and converts to image
/**
 * This class is used to create a LidarClient object to receive Lidar data from the server
 */
class LidarClient(ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) : AbstractClient(ip, port, timeToTimeout){
    /**
     * Send message to server requesting to turn on Lidar
     */
    fun sendStartLidar(){
        val id = MessageType.LIDAR.value
        val size = 2
        val start = 1 // What should this be?
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start.toShort())
        sendDataToServer(id, buffer.array())
    }
    /**
     * Send request to server to start sending Lidar data
     */
    fun sentRequestLidarData(){
        val id = MessageType.REQ_LIDAR.value
        val size = 0
        val buffer = ByteBuffer.allocate(size)
        sendDataToServer(id, buffer.array())
    }

    fun receiveLidarData(){
        val data = fetch()
        println("Message received: ${String(data)}")
    }
}