package org.example
import java.lang.Thread.sleep
import java.net.Socket
import java.nio.ByteBuffer

class Client {
    fun start() {
        val ip = "localhost"
        val port = 12345
        val socket = Socket(ip, port)
        println("Connected to server")
        val frameBufferSize = ByteBuffer.allocate(4)
        socket.getInputStream().read(frameBufferSize.array())

        val frameSize = frameBufferSize.int

        //val frameBuffer = ByteArray(frameSize)
        val frameBuffer = ByteBuffer.allocate(frameSize)
        var bytesRead = 0

        while (bytesRead < frameSize){
            bytesRead += socket.getInputStream().read(frameBuffer.array(), bytesRead, frameSize - bytesRead)
        }
        val message = String(frameBuffer.array())
        println("Received message: $message")

        socket.close()
    }
}
fun main(args: Array<String>) {
    val client : Client = Client()
    client.start()
}