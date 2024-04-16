package org.example
import java.lang.Thread.sleep
import java.net.ServerSocket
import java.nio.ByteBuffer

class Server {
    fun start() {
        val port = 12345
        val serverSocket = ServerSocket(port)
        println("Server started on port $port")
        val socket = serverSocket.accept()
        println("Client connected")
        sleep(1000)
        val message = "Hello from the server!"
        val frameSize = message.length
        val frameBufferSize = ByteBuffer.allocate(4)
        frameBufferSize.putInt(frameSize)
        socket.getOutputStream().write(frameBufferSize.array())
        socket.getOutputStream().write(message.toByteArray())
        socket.close()
        serverSocket.close()
    }
}
fun main(args: Array<String>) {
    val server : Server = Server()
    server.start()
}