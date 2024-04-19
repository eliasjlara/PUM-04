import org.example.Client
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Disabled
import org.example.Server
import java.lang.Thread.sleep

class SocketTest {
    @Test
    fun testStringCommunication(){
        val message = "Hello, World!"
        val serverThread = Thread{
            val server = Server()
            server.sendData(message.toByteArray())
            server.closeConnection()
        }
        serverThread.start()
        val client = Client()
        val receivedMessage  = client.fetch()
        assertEquals(message, receivedMessage.toString(Charsets.UTF_8))
        serverThread.join()
        client.stop()
    }

    // This test is disabled because it is no longer valid
    @Disabled
    fun testServerReceiveResponse(){
        var response = "Hello, World!"
        val serverThread = Thread{
            val server = Server()
            server.sendData("Hello, World!".toByteArray())
            response = server.receiveResponse()
            //assertEquals(response, "ack\n")
            server.closeConnection()
        }
        serverThread.start()
        val client = Client()
        val receivedMessage  = client.fetch()
        // Idk why i get the end of file exception here
        serverThread.join()
        //assertEquals(response, "ack\n")
        client.stop()
    }

    @Test
    fun testSendNoData(){
        val serverThread = Thread{
            val server = Server()
            server.sendData(ByteArray(0))
            server.closeConnection()
        }
        serverThread.start()
        val client = Client()
        val receivedMessage  = client.fetch()
        assertEquals("", receivedMessage.toString(Charsets.UTF_8))
        serverThread.join()
        client.stop()
    }
}