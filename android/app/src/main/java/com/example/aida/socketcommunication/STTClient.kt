package com.example.aida.socketcommunication

import java.nio.ByteBuffer
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
        val start = Instructions.ON.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start)
        sendDataToServer(id, buffer.array())
    }
    /**
     * Send message to server requesting to turn on STT-transcription
     */
    fun sendStartSTT(){
        val id = MessageType.STT.value
        val size = 2
        val start = Instructions.ON.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(start)
        sendDataToServer(id, buffer.array())
    }

    /**
     * Send message to server requesting to stop microphone
     */
    fun sendStopMic(){
        val id = MessageType.MIC.value
        val size = 2
        val stop = Instructions.OFF.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(stop)
        sendDataToServer(id, buffer.array())
    }

    /**
     * Send message to server requesting to stop STT-transcription
     */
    fun sendStopSTT() {
        val id = MessageType.STT.value
        val size = 2
        val stop = Instructions.OFF.value
        val buffer = ByteBuffer.allocate(size)
        buffer.putShort(stop)
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
     * Receives STT data from server and converts to string
     * @return String of the STT data
     */
    fun receiveSTTData() : String{
        val data = fetch()
        return data.toString(Charsets.UTF_8)
    }
    /**
     *  Gets new data and prints the result to terminal
     *  -- For debug purposes
     */
    fun debugReceiveSTTData(){
        val message = fetch()
        println("Message received from STT: "+ message.toString(Charsets.UTF_8))
    }
}