package org.example
import java.lang.Thread.sleep
import java.net.Socket
import java.nio.ByteBuffer
import javax.imageio.ImageIO
import javax.swing.JFrame
import javax.swing.ImageIcon
import javax.swing.JLabel

class Client (ip : String = "localhost", port : Int = 12345) {
    val frame = JFrame("Image Display")
    private val label = JLabel()
    private val socket : Socket = Socket(ip, port)

    fun displayImageFromByteArray(imageData: ByteArray) {
        // Read the image from the byte array
        val image = ImageIO.read(imageData.inputStream())

        label.icon = ImageIcon(image)
        frame.contentPane.add(label)

        // Pack the frame and set it visible
        frame.pack()
        frame.isVisible = true
    }
    private fun sendResponse(response: String = "ack\n") {
        socket.getOutputStream().write(response.toByteArray())
    }

    fun getFrameSize() : Int {
        val frameBufferSize = ByteBuffer.allocate(4)
        socket.getInputStream().read(frameBufferSize.array())
        return frameBufferSize.int
    }

    fun getFrame(frameSize: Int) : ByteArray {
        val frameBuffer = ByteBuffer.allocate(frameSize)
        var bytesRead = 0

        while (bytesRead < frameSize) {
            bytesRead += socket.getInputStream().read(frameBuffer.array(), bytesRead, frameSize - bytesRead)
        }
        return frameBuffer.array()
    }

    fun fetch() : ByteArray{
        val frameSize = getFrameSize()
        sendResponse()
        return getFrame(frameSize)
    }

    fun stop() {
        socket.close()
    }

    fun receiveImage() {
        val imageData = fetch()
        displayImageFromByteArray(imageData)
    }
}
fun main(args: Array<String>) {
    val client = Client()
    client.frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    while (true){
        client.receiveImage()
    }
    client.stop()
}