package com.example.aida.socketcommunication

import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Abstract class for client that connects to server located on AIDA.
 * The client can send and receive messages over socket
 */
abstract class AbstractClient(ip : String = "localhost", port : Int = 12345) {
    val socket : Socket = Socket(ip, port)

    /**
     * Get the header of the message sent over socket
     * @return Pair with id of the message and size of message
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
     * Get the latest response from server
     * @return The message sent over socket
     */
    fun fetch() : ByteArray{
        val (id, size) = getHeader()
        //sendResponse()
        return getBody(size)
    }

    /**
     * Closes the socket connection
     */
    fun stop() {
        socket.close()
    }
}