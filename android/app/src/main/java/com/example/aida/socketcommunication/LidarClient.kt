package com.example.aida.socketcommunication

import android.graphics.BitmapFactory
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import java.nio.ByteBuffer
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
        val start = Instructions.ON.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start)
        sendDataToServer(id, buffer.array())
    }
    /**
     * Send message to server requesting to turn off Lidar
     */
    fun sendStopLidar() {
        val id = MessageType.LIDAR.value
        val size = 2
        val stop = Instructions.OFF.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(stop)
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

    /**
     * Receives lidar data from server and converts to Image
     * @return ImageBitmap? of the video data
     */
    fun receiveLidarData() : ImageBitmap?{
        val data = fetch()
        return BitmapFactory.decodeByteArray(data, 0, data.size)
            ?.asImageBitmap()
    }
}