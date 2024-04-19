package org.example
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder
import javax.imageio.ImageIO
import javax.swing.JFrame
import javax.swing.ImageIcon
import javax.swing.JLabel

class Client (ip : String = "localhost", port : Int = 12345) {
    val frame = JFrame("Image Display")
    private val label = JLabel()
    private val socket : Socket = Socket(ip, port)
    //private var id : Short = 0
    //private var size : Int = 1024



    private fun sendResponse(response: String = "ack\n") {
        socket.getOutputStream().write(response.toByteArray())
    }

    fun getHeader() : Pair<Short, Int>{
        var headerBuffer = ByteBuffer.allocate(6)
        socket.getInputStream().read(headerBuffer.array())
        headerBuffer = ByteBuffer.wrap(headerBuffer.array()).order(ByteOrder.BIG_ENDIAN)


        val id  = headerBuffer.short
        val size = headerBuffer.int

        println("ID: $id,Size: $size")
        return Pair(id, size)

    }

    fun getFrame(size : Int) : ByteArray {
        val frameBuffer = ByteBuffer.allocate(size)
        var bytesRead = 0

        while (bytesRead < size) {
            bytesRead += socket.getInputStream().read(frameBuffer.array(), bytesRead, size - bytesRead)
        }
        return frameBuffer.array()
    }

    fun fetch() : ByteArray{
        val (id, size) = getHeader()
        //sendResponse()
        return getFrame(size)
    }

    fun stop() {
        socket.close()
    }

    fun receiveImage(imagePainter: ImagePainter) {
        val imageData = fetch()
        imagePainter.displayImageFromByteArray(imageData)
    }

    fun sendStartCamera(){
        val id = 1
        val size = 2
        val frameBufferSize = ByteBuffer.allocate(6)
        frameBufferSize.putShort(id.toShort())
        frameBufferSize.putInt(size)
        frameBufferSize.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(frameBufferSize.array())
        val start = 1
        val startBuffer = ByteBuffer.allocate(2)
        startBuffer.putShort(start.toShort())
        startBuffer.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(startBuffer.array())
    }

    fun sendGetVideo(){
        val id = 6
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
    val client = Client()
    val imagePainter = ImagePainter()
    imagePainter.setDefaultCloseOperation()

    client.sendStartCamera()
    client.sendGetVideo()
    client.frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    while (true){
        client.receiveImage(imagePainter)
    }
    client.stop()
}