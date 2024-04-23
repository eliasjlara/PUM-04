package com.example.aida.socketcommunication
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Client class that connects to server to receive
 * video data
 */

class VideoClient (ip : String = "localhost", port : Int = 12345) : AbstractClient(ip, port){
    /**
     * Receives the message and shows it using helper method
     * in SocketHelpers.kt
     */
    /*
    fun receiveImage(imagePainter: ImagePainter) {
        val imageData = fetch()
        imagePainter.displayImageFromByteArray(imageData)
    }*/

    /**
     * Sends a request to start the camera to the server
     */
    fun sendStartCamera(){
        val id = MessageType.CAMERA.value
        val size = 2
        val frameBufferSize = ByteBuffer.allocate(6)
        frameBufferSize.putShort(id.toShort())
        frameBufferSize.putInt(size)
        frameBufferSize.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(frameBufferSize.array())
        val start = 1 // What should this be?
        val startBuffer = ByteBuffer.allocate(2)
        startBuffer.putShort(start.toShort())
        startBuffer.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(startBuffer.array())
    }

    /**
     * Sends a request to server to start sending video data
     */
    fun sendGetVideo(){
        val id = MessageType.REQ_VIDEO_FEED.value
        val size = 0
        val frameBufferSize = ByteBuffer.allocate(6)
        frameBufferSize.putShort(id.toShort())
        frameBufferSize.putInt(size)
        frameBufferSize.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(frameBufferSize.array())
    }
}
fun main(args: Array<String>) {
    //val client = Client("192.168.37.50", 9000)
    val videoClient = VideoClient()
    //val imagePainter = ImagePainter()
    //imagePainter.setDefaultCloseOperation()

    videoClient.sendStartCamera()
    videoClient.sendGetVideo()
    //videoClient.frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    while (true){
        //videoClient.receiveImage(imagePainter)
        videoClient.fetch()
    }
    videoClient.stop()
}