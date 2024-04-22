package com.example.aida.socketcommunication
import java.io.File
import java.net.ServerSocket
import java.nio.ByteBuffer
import java.io.BufferedReader
import java.io.InputStreamReader
import java.nio.ByteOrder

/**
 * Server class that listens for incoming connections
 * and sends data to the client
 * -- For testing purposes, not used in application
 */

class Server (port: Int = 12345) {
    private val serverSocket = ServerSocket(port)
    private val socket = serverSocket.accept()

   fun receiveResponse() : String {
        val response = BufferedReader(InputStreamReader(socket.getInputStream())).readLine()
        return response
    }
    fun sendData(id: Short, data: ByteArray) {
        //val id = MessageType.STT.value
        val frameSize = data.size
        val frameBufferSize = ByteBuffer.allocate(6)
        frameBufferSize.putShort(id)
        frameBufferSize.putInt(frameSize)

        frameBufferSize.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(frameBufferSize.array())
        socket.getOutputStream().write(data)
    }

    fun closeConnection() {
        socket.close()
        serverSocket.close()
    }

    fun getHeader() : Pair<Short, Int>{
        var headerBuffer = ByteBuffer.allocate(6)
        socket.getInputStream().read(headerBuffer.array())
        headerBuffer = ByteBuffer.wrap(headerBuffer.array()).order(ByteOrder.BIG_ENDIAN)
        val id  = headerBuffer.short
        val size = headerBuffer.int

        return Pair(id, size)
    }

    /**
     * Get the body of message sent over socket
     * @return The message as a ByteArray
     */
    fun getBody(size : Int) : ByteArray {
        val frameBuffer = ByteBuffer.allocate(size)
        var bytesRead = 0

        while (bytesRead < size) {
            bytesRead += socket.getInputStream().read(frameBuffer.array(), bytesRead, size - bytesRead)
        }
        return frameBuffer.array()
    }
    fun receiveMessage() {
        val (id, size) = getHeader()
        val message = getBody(size)
        //if(id == MessageType.REQ_STT.value) {
        println("Message received: ${String(message)}")
        //}
        //sendData(id, "Hello, World!".toByteArray())
    }
}
fun main(args: Array<String>) {
    val server: Server = Server()
    val path = "C:\\Users\\Programmer\\IdeaProjects\\PUM-04\\SocketExperiments\\SocketCommunication\\src\\main\\kotlin\\socketbild.jpeg"
    val file = File(path)
    val message = file.readBytes()
    server.receiveMessage()
    server.receiveMessage()
    while (true) {

        server.sendData(MessageType.CAMERA.value,message)
    }
    server.closeConnection()
}