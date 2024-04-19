package org.example
import java.io.File
import java.net.ServerSocket
import java.nio.ByteBuffer
import java.io.BufferedReader
import java.io.InputStreamReader
import java.nio.ByteOrder

class Server (port: Int = 12345) {
    private val serverSocket = ServerSocket(port)
    private val socket = serverSocket.accept()

   fun receiveResponse() : String {
        val response = BufferedReader(InputStreamReader(socket.getInputStream())).readLine()
        return response
    }
    fun sendData(data: ByteArray) {
        val id = 1
        val frameSize = data.size
        val frameBufferSize = ByteBuffer.allocate(6)
        frameBufferSize.putShort(id.toShort())
        frameBufferSize.putInt(frameSize)

        frameBufferSize.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(frameBufferSize.array())
        socket.getOutputStream().write(data)
    }

    fun closeConnection() {
        socket.close()
        serverSocket.close()
    }
}
fun main(args: Array<String>) {
    val server: Server = Server()
    val path = "C:\\Users\\Programmer\\IdeaProjects\\PUM-04\\SocketExperiments\\SocketCommunication\\src\\main\\kotlin\\socketbild.jpeg"
    val file = File(path)
    val message = file.readBytes()

    while (true) {
        server.sendData(message)
    }
    server.closeConnection()
}