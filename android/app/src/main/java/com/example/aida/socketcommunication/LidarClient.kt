package com.example.aida.socketcommunication

//import kotlin.math.radians
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import java.nio.ByteBuffer
import java.nio.ByteOrder
import kotlin.math.cos
import kotlin.math.sin

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
    /*fun receiveLidarData() : ImageBitmap?{
        val data = fetch()
        return BitmapFactory.decodeByteArray(data, 0, data.size)
            ?.asImageBitmap()
    }*/
    @OptIn(ExperimentalUnsignedTypes::class)
    fun receiveLidarData() : ImageBitmap{
        println("AAAAHHH !!")
        getHeader()
        //val data = fetch()
        val byteArray = ByteArray(1440)
        var bytesRead = 0

        while (bytesRead < 1440) {
            bytesRead += socket.getInputStream().read(byteArray, bytesRead, 1440 - bytesRead)
        }
        val intArray = IntArray(byteArray.size / 4)
        val byteBuffer = ByteBuffer.wrap(byteArray).order(ByteOrder.BIG_ENDIAN)
        val intValue = 2147483647
        for (i in intArray.indices) {
            intArray[i] = byteBuffer.int
            print("AAAAHHH: ${intArray[i]} ")
        }
        //val dataAsInts = ByteBuffer.wrap(byteArray).order(ByteOrder.BIG_ENDIAN).asIntBuffer()
        // Bytearray with 360 values
        println("AAAAHHH 3")
        val a = createLiDARBitmap(intArray, 640, 640)
        println("AAAAHHH 10")
        return a.asImageBitmap()
    }

    /**
     * Create a bitmap from the lidar values received
     * @param lidarValues The lidar values received from the server
     * @param width The width of the bitmap
     * @param height The height of the bitmap
     * @return Bitmap of the lidar values
     */
    private fun createLiDARBitmap(lidarValues: IntArray, width: Int, height: Int): Bitmap {
        println("AAAAHHH 4")
        val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565)
        val canvas = Canvas(bitmap)
        canvas.drawColor(Color.WHITE)
        println("AAAAHHH 5")
        val MAX_LIDAR_DISTANCE = 5000  // Max distance of lidar in mm
        val centerX = width / 2f
        val centerY = height / 2f
        //val radius = min(width, height) / 2


        val paint = Paint()
        paint.color = Color.BLACK
        paint.strokeWidth = 3f
        println("AAAAHHH 6")

        for ((index, distance) in lidarValues.withIndex()) {
            println("AAAAHHH 7")
            val angle = Math.toRadians(index.toDouble())
            val normalizedDistanceX = distance.toFloat() / (MAX_LIDAR_DISTANCE/centerX)
            val normalizedDistanceY = distance.toFloat() / (MAX_LIDAR_DISTANCE/centerY)

            val x = centerX + normalizedDistanceX * cos(angle).toFloat()
            val y = centerY + normalizedDistanceY * sin(angle).toFloat()

            if (x >= 0 && x < width && y >= 0 && y < height) {
                canvas.drawPoint(x, y, paint)
                println("AAAAHHH 8")
            }
        }
        println("AAAAHHH 9")
        return bitmap
    }
}