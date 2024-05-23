package com.example.aida.socketcommunication
import java.io.BufferedReader
import java.io.File
import java.io.InputStreamReader
import java.net.ServerSocket
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Server class that listens for incoming connections
 * and sends data to the client
 * -- For testing purposes, not used in application
 */

class Server (port: Int = 12345) {
    private val serverSocket = ServerSocket(port)
    private val socket = serverSocket.accept()

    /**
     * Send data to the client
     * @param id The id of the message
     * @param data The data to be sent
     * For testing purposes
     */
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

    /**
     * Close the connection to the socket
     *
     */
    fun closeConnection() {
        socket.close()
        serverSocket.close()
    }

    /**
     * Get the header of the message sent over socket
     * @return Pair with id of the message and size of message
     * For testing purposes
     */
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
     * For testing purposes
     */
    fun getBody(size : Int) : ByteArray {
        val frameBuffer = ByteBuffer.allocate(size)
        var bytesRead = 0

        while (bytesRead < size) {
            bytesRead += socket.getInputStream().read(frameBuffer.array(), bytesRead, size - bytesRead)
        }
        return frameBuffer.array()
    }

    /**
     * Only for debug
     */
    fun receiveMessage() {
        val (id, size) = getHeader()
        val message = getBody(size)
        //if(id == MessageType.REQ_STT.value) {
        println("Message received: ${String(message)}")
        //}
        //sendData(id, "Hello, World!".toByteArray())
    }
}