package org.example
import java.io.DataInputStream
import java.io.File
import java.lang.Thread.sleep
import java.net.ServerSocket
import java.nio.ByteBuffer
import java.io.BufferedReader
import java.io.InputStreamReader

class Server (port: Int = 12345) {
    private val serverSocket = ServerSocket(port)
    private val socket = serverSocket.accept()

   fun receiveResponse() : String {
        val response = BufferedReader(InputStreamReader(socket.getInputStream())).readLine()
        //val response = DataInputStream(socket.getInputStream()).readUTF()
        return response
    }
    fun sendData(data: ByteArray) {
        val frameSize = data.size
        val frameBufferSize = ByteBuffer.allocate(4)

        frameBufferSize.putInt(frameSize)
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
    //val path = "C:\\Users\\Programmer\\IdeaProjects\\PUM-04\\SocketExperiments\\SocketCommunication\\src\\main\\kotlin\\socketbild.jpeg"
    //val file = File(path)
    //val message = file.readBytes()
    val message = "Hello, World!".toByteArray()
    while (true) {
        server.sendData(message)
        val response = server.receiveResponse()
        if (response == "ack\n") {
            println("Client received the image")
        } else {
            println("Client did not received the image")
            break
        }
    }
    server.closeConnection()
}