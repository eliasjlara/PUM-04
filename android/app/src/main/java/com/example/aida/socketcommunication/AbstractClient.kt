package com.example.aida.socketcommunication

import java.net.InetSocketAddress
import java.net.Socket
import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Abstract class for client that connects to server located on AIDA.
 * The client can send and receive messages over socket
 */
abstract class AbstractClient(ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) {
    // TODO - should this be lateinit? As we are using specific init function
    private val socket: Socket

    init {
        socket = Socket().apply {
            connect(InetSocketAddress(ip, port), timeToTimeout)
        }
    }
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
    private fun getBody(size : Int) : ByteArray {
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
    protected fun fetch() : ByteArray{
        val (id, size) = getHeader()
        //sendResponse()
        return getBody(size)
    }

    /**
     * Function sends a message to the server
     * @param id Id explains to server the what type of message @see MessageType
     * @param data The data to be sent to the server
     */
    protected fun sendDataToServer(id: Short, data: ByteArray) {
        val frameSize = data.size
        val header = ByteBuffer.allocate(6)
        header.order(ByteOrder.BIG_ENDIAN)
        header.putShort(id)
        header.putInt(frameSize)
        header.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(header.array())
        socket.getOutputStream().write(data)
    }

    /**
     * Closes the socket connection
     */
    fun stop() {
        socket.close()
    }
}