package org.example
import java.lang.Thread.sleep
import java.net.Socket
import java.nio.ByteBuffer
import javax.imageio.ImageIO
import javax.swing.JFrame
import javax.swing.ImageIcon
import javax.swing.JLabel

class Client {
    val frame = JFrame("Image Display")
    private val label = JLabel()

    private fun displayImageFromByteArray(imageData: ByteArray) {
        // Read the image from the byte array
        val image = ImageIO.read(imageData.inputStream())

        label.icon = ImageIcon(image)
        frame.contentPane.add(label)

        // Pack the frame and set it visible
        frame.pack()
        frame.isVisible = true
    }
    fun start() {
        val ip = "localhost"
        val port = 12345
        val socket = Socket(ip, port)
        println("Connected to server")
        while (true){
            val frameBufferSize = ByteBuffer.allocate(4)
            socket.getInputStream().read(frameBufferSize.array())

            val frameSize = frameBufferSize.int
            //socket.getOutputStream().write("ack\n".toByteArray())
            val frameBuffer = ByteBuffer.allocate(frameSize)
            var bytesRead = 0

            while (bytesRead < frameSize){
                bytesRead += socket.getInputStream().read(frameBuffer.array(), bytesRead, frameSize - bytesRead)
            }
            displayImageFromByteArray(frameBuffer.array())

        }
        socket.close()
    }
}
fun main(args: Array<String>) {
    val client : Client = Client()
    client.frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE

    client.start()
}