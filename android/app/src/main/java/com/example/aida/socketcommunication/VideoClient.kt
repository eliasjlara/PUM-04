package com.example.aida.socketcommunication
import java.nio.ByteBuffer

// TODO - Add function that receives video data and converts to image
// TODO - instead of using fetch() function and converting in main activity
/**
 * Client class that connects to server to receive
 * video data
 */
class VideoClient (ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) : AbstractClient(ip, port, timeToTimeout){
    /**
     * Sends a request to start the camera to the server
     */
    fun sendStartCamera(){
        val id = MessageType.CAMERA.value
        val size = 2
        val start = 1
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start.toShort())
        sendDataToServer(id, buffer.array())
    }

    /**
     * Sends a request to server to start sending video data
     */
    fun sendGetVideo(){
        val id = MessageType.REQ_VIDEO_FEED.value
        val size = 0
        val buffer = ByteBuffer.allocate(size)
        sendDataToServer(id, buffer.array())
    }
    // TODO - Use fetch to create a image instead of doing it in the application
}