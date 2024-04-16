package org.example
import java.io.File
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
        val path = "C:\\Users\\alexa\\IdeaProjects\\PUM-04\\SocketExperiments\\SocketCommunication\\src\\main\\kotlin\\socketbild.jpeg"
        val file = File(path)
        val message = file.readBytes()
        val frameSize = message.size
        val frameBufferSize = ByteBuffer.allocate(4)

        //val message = "Hello from the server!"
        //val frameSize = message.length
        //val frameBufferSize = ByteBuffer.allocate(4)
        frameBufferSize.putInt(frameSize)
        socket.getOutputStream().write(frameBufferSize.array())
        //socket.getOutputStream().write(message.toByteArray())
        socket.getOutputStream().write(message)
        socket.close()
        serverSocket.close()
    }
}
fun main(args: Array<String>) {
    val server : Server = Server()
    server.start()
}