package org.example
import java.lang.Thread.sleep
import java.net.Socket
import java.nio.ByteBuffer
import javax.imageio.ImageIO
import javax.swing.JFrame
import javax.swing.ImageIcon
import javax.swing.JLabel

class Client {
    private fun displayImageFromByteArray(imageData: ByteArray) {
        try {
            // Read the image from the byte array
            val image = ImageIO.read(imageData.inputStream())

            // Create a JFrame to display the image
            val frame = JFrame("Image Display")
            frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE

            // Create a JLabel to hold the image
            val label = JLabel()
            label.icon = ImageIcon(image)

            // Add the JLabel to the frame
            frame.contentPane.add(label)
            frame.setSize(image.width, image.height)

            // Pack the frame and set it visible
            frame.pack()
            frame.isVisible = true
        } catch (ex: Exception) {
            ex.printStackTrace()
        }
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

            //val frameBuffer = ByteArray(frameSize)
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
    client.start()
}