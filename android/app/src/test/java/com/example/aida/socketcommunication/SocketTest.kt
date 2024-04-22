import com.example.aida.socketcommunicaion.MessageType
import com.example.aida.socketcommunicaion.STTClient
import org.junit.jupiter.api.Assertions.assertEquals
import com.example.aida.socketcommunicaion.Server
import org.junit.jupiter.api.*
import java.lang.Thread.sleep

class SocketTest {
    @Test
    fun testGetHeader(){
        val serverThread = Thread {
            val server = Server()
            server.sendData(MessageType.STT.value, "Hello, World!".toByteArray())
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = STTClient()
        client.sendStartSTT()
        val (id, size) = client.getHeader()
        assertEquals(MessageType.STT.value, id)
        assertEquals(13, size)
        client.stop()
        serverThread.join()
    }

    @Test
    fun testStringCommunication(){
        val serverThread = Thread {
            val server = Server()
            server.receiveMessage()
            server.sendData(MessageType.STT.value,"Hello, World!".toByteArray())
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = STTClient()
        client.sendStartSTT()
        val receivedMessage  = client.fetch()
        assertEquals("Hello, World!", receivedMessage.toString(Charsets.UTF_8))
        client.stop()
        serverThread.join()
    }
    @Test
    fun testSendNoData(){
        val serverThread = Thread{
            val server = Server()
            server.receiveMessage()
            server.sendData(MessageType.STT.value,ByteArray(0))
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = STTClient()
        client.sendStartSTT()
        val receivedMessage  = client.fetch()
        assertEquals("", receivedMessage.toString(Charsets.UTF_8))
        client.stop()
        serverThread.join()
    }
}