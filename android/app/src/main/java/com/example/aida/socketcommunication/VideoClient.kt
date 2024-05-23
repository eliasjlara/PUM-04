package com.example.aida.socketcommunication
import android.graphics.BitmapFactory
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import java.nio.ByteBuffer

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
        val start = Instructions.ON.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start)
        sendDataToServer(id, buffer.array())
    }
    /**
     * Sends a request to stop the camera feed to the server
     */
    fun sendStopCamera() {
        val id = MessageType.CAMERA.value
        val size = 2
        val stop = Instructions.OFF.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(stop)
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

    /**
     * Receives video data from server and converts to Image
     * @return ImageBitmap? of the video data
     */
    fun receiveVideoData() : ImageBitmap?{
        val data = fetch()
        return BitmapFactory.decodeByteArray(data, 0, data.size)
            ?.asImageBitmap()

    }
}