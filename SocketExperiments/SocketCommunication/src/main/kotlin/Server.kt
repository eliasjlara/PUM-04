package org.example
import java.io.DataInputStream
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
        while (true){
            val path = "C:\\Users\\Programmer\\IdeaProjects\\PUM-04\\SocketExperiments\\SocketCommunication\\src\\main\\kotlin\\socketbild.jpeg"
            val file = File(path)
            val message = file.readBytes()
            val frameSize = message.size
            val frameBufferSize = ByteBuffer.allocate(4)

            frameBufferSize.putInt(frameSize)
            socket.getOutputStream().write(frameBufferSize.array())
            socket.getOutputStream().write(message)



            val response = DataInputStream(socket.getInputStream()).readUTF()
            if(response != "ack\n"){
                println("Client did not received the image")
                break
            }
            //socket.getInputStream().readUTF(response.array())
            //if (!response.array().contentEquals("ack\n".toByteArray())){
            //if(!response.asCharBuffer().equals("ack\n")){
            //    println("Client did not receive the image")
            //    break
            //}
        }
        socket.close()
        serverSocket.close()
    }
}
fun main(args: Array<String>) {
    val server : Server = Server()
    server.start()
}