package com.example.aida.socketcommunication

import java.nio.ByteBuffer
import java.nio.ByteOrder

/**
 * Client class that connects to server and sends messages
 * to get STT data
 */
class STTClient (ip : String = "localhost", port : Int = 12345) : AbstractClient(ip, port){
    /**
     * Send message to server requesting to turn on STT-transcription
     */
    fun sendStartSTT(){
        val id = MessageType.STT.value
        val size = 2
        val frameBufferSize = ByteBuffer.allocate(6)
        frameBufferSize.putShort(id.toShort())
        frameBufferSize.putInt(size)
        frameBufferSize.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(frameBufferSize.array())
        val start = 1 // What should this be?
        val startBuffer = ByteBuffer.allocate(2)
        startBuffer.putShort(start.toShort())
        startBuffer.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(startBuffer.array())
    }

    /**
     * Send request to server to start sending STT data
     */
    fun sendRequestSTTData(){
        val id = MessageType.REQ_STT.value
        val size = 0
        val frameBufferSize = ByteBuffer.allocate(6)
        frameBufferSize.putShort(id.toShort())
        frameBufferSize.putInt(size)
        frameBufferSize.order(ByteOrder.BIG_ENDIAN)
        socket.getOutputStream().write(frameBufferSize.array())
    }

    /**
     *  Gets new data and prints the result to terminal
     *  -- For debug purposes
     */
    fun receiveSTTData(){
        val message = fetch()
        println("Message received: $String(message)")
    }
}

fun main(args: Array<String>){
    val client = STTClient()
    client.sendStartSTT()
    client.sendRequestSTTData()

    while (true){
        client.receiveSTTData()
    }
    client.stop()
}