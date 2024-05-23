package com.example.aida.socketcommunication

import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import com.example.aida.viewmodels.LidarPainter
import java.nio.ByteBuffer

/**
 * This class is used to create a LidarClient object to receive Lidar data from the server
 */
class LidarClient(ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) : AbstractClient(ip, port, timeToTimeout){
    // Create a LidarPainter object to convert Lidar data to Bitmap
    private val lidarPainter = LidarPainter(640, 640, 5000)

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
    /*fun receiveLidarData() : ImageBitmap?{
        val data = fetch()
        return BitmapFactory.decodeByteArray(data, 0, data.size)
            ?.asImageBitmap()
    }*/
    // This function is used to receive Lidar data from the server and convert it to ImageBitmap
    // This works - But should test if I can use fetch instead
    /*fun receiveLidarData() : ImageBitmap{
        getHeader()
        //val data = fetch()
        val byteArray = ByteArray(1440)
        var bytesRead = 0

        while (bytesRead < 1440) {
            bytesRead += socket.getInputStream().read(byteArray, bytesRead, 1440 - bytesRead)
        }
        val intArray = IntArray(byteArray.size / 4)
        val byteBuffer = ByteBuffer.wrap(byteArray).order(ByteOrder.BIG_ENDIAN)
        for (i in intArray.indices) {
            intArray[i] = byteBuffer.int
        }
        val a = lidarPainter.createLiDARBitmap(intArray)
        return a.asImageBitmap()
    }*/
    /**
     * Receives lidar data from server and converts to Image
     * @return ImageBitmap of the Lidar data
     */
    fun receiveLidarData() : ImageBitmap {
        val data = fetch()
        val intArray = IntArray(data.size / 4)
        val byteBuffer = ByteBuffer.wrap(data).order(java.nio.ByteOrder.BIG_ENDIAN)
        for (i in intArray.indices) {
            intArray[i] = byteBuffer.int
        }
        val a = lidarPainter.createLiDARBitmap(intArray)
        return a.asImageBitmap()
    }

}