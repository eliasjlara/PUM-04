package com.example.aida.socketcommunication

import java.nio.ByteBuffer
// TODO - Add function that receives STT data and converts to string
// TODO - instead of using fetch() function and converting in main activity
/**
 * Client class that connects to server and sends messages
 * to get STT data
 */
class STTClient (ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) : AbstractClient(ip, port, timeToTimeout){
    /**
     * Send message to server requesting to turn on microphone
     */
    fun sendStartMic(){
        val id = MessageType.MIC.value
        val size = 2
        val start = 1
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start.toShort())
        sendDataToServer(id, buffer.array())
    }
    /**
     * Send message to server requesting to turn on STT-transcription
     */
    fun sendStartSTT(){
        val id = MessageType.STT.value
        val size = 2
        val start = 1
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start.toShort())
        sendDataToServer(id, buffer.array())
    }

    /**
     * Send request to server to start sending STT data
     */
    fun sendRequestSTTData(){
        val id = MessageType.REQ_STT.value
        val size = 0
        val buffer = ByteBuffer.allocate(size)
        sendDataToServer(id, buffer.array())
    }

    /**
     *  Gets new data and prints the result to terminal
     *  -- For debug purposes
     */
    fun receiveSTTData(){
        val message = fetch()
        println("Message received from STT: "+ message.toString(Charsets.UTF_8))
    }
}