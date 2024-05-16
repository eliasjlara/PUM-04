package com.example.aida.socketcommunication

import org.junit.Assert.assertEquals
import org.junit.Test
import java.io.File
import java.lang.Thread.sleep
import java.nio.ByteBuffer

class SocketTest {

    /**
     * Test that the header is received correctly in the STTClient
     */
    @Test
    fun testGetHeaderSTT(){
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

    /**
     * Test that the header is received correctly in the LidarClient
     */
    @Test
    fun testGetHeaderLidar(){
        val serverThread = Thread {
            val server = Server()
            // TODO - Send a message with lidar data
            server.sendData(MessageType.LIDAR.value, "Hello, World!".toByteArray())
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = LidarClient()
        client.sendStartLidar()
        client.sentRequestLidarData()
        val (id, size) = client.getHeader()
        assertEquals(MessageType.LIDAR.value, id)
        assertEquals(13, size)
        client.stop()
        serverThread.join()
    }

    /**
     * Test that the header is received correctly in the VideoClient
     */
    @Test
    fun testGetHeaderVideo(){
        val serverThread = Thread {
            val server = Server()
            // TODO - Send a message with video data
            server.sendData(MessageType.CAMERA.value, "Hello, World!".toByteArray())
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = VideoClient()
        client.sendStartCamera()
        client.sendGetVideo()
        val (id, size) = client.getHeader()
        assertEquals(MessageType.CAMERA.value, id)
        assertEquals(13, size)
        client.stop()
        serverThread.join()
    }

    /**
     * Test that string communication works for STTClient
     */
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

    /**
     * Check that the STT client can receive an empty message
     */
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

    /**
     * Check that the STT client can receive a message with a single character
     */
    @Test
    fun testSendSingleChar(){
        val serverThread = Thread{
            val server = Server()
            server.receiveMessage()
            server.sendData(MessageType.STT.value,"A".toByteArray())
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = STTClient()
        client.sendStartSTT()
        val receivedMessage  = client.fetch()
        assertEquals("A", receivedMessage.toString(Charsets.UTF_8))
        client.stop()
        serverThread.join()
    }

    /**
     * Check that StartMic message is sent correctly
     */
    @Test
    fun testSendStartMic(){
        val serverThread = Thread{
            val server = Server()
            sleep(500)
            val (id, size) = server.getHeader()
            assertEquals(MessageType.MIC.value, id)
            assertEquals(2, size)
            val message = server.getBody(size)
            assertEquals(1, ByteBuffer.wrap(message).short)

            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = STTClient()
        client.sendStartMic()
        client.stop()
        serverThread.join()
    }

    /**
     * Check that StartSTT message is sent correctly
     */
    @Test
    fun testSendStartSTT(){
        val serverThread = Thread{
            val server = Server()
            sleep(500)
            val (id, size) = server.getHeader()
            val message = server.getBody(size)
            assertEquals(MessageType.STT.value, id)
            assertEquals(2, size)
            assertEquals(1, ByteBuffer.wrap(message).short)

            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = STTClient()
        client.sendStartSTT()
        client.stop()
        serverThread.join()
    }

    /**
     * Check that ReqSTT message is sent correctly
     */
    @Test
    fun testSendReqSTT(){
        val serverThread = Thread{
            val server = Server()
            sleep(500)
            val (id, size) = server.getHeader()
            assertEquals(MessageType.REQ_STT.value, id)
            assertEquals(0, size)
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = STTClient()
        client.sendRequestSTTData()
        client.stop()
        serverThread.join()
    }

    /**
     * Check that StartLidar message is sent correctly
     */
    @Test
    fun testSendStartLidar(){
        val serverThread = Thread{
            val server = Server()
            sleep(500)
            val (id, size) = server.getHeader()
            val message = server.getBody(size)
            assertEquals(MessageType.LIDAR.value, id)
            assertEquals(2, size)
            assertEquals(1, ByteBuffer.wrap(message).short)
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = LidarClient()
        client.sendStartLidar()
        client.stop()
        serverThread.join()
    }

    /**
     * Check that ReqLidar message is sent correctly
     */
    @Test
    fun testSendReqLidar(){
        val serverThread = Thread{
            val server = Server()
            sleep(500)
            val (id, size) = server.getHeader()
            val message = server.getBody(size)
            assertEquals(MessageType.REQ_LIDAR.value, id)
            assertEquals(0, size)
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = LidarClient()
        client.sentRequestLidarData()
        client.stop()
        serverThread.join()
    }

    /**
     * Check that StartCamera message is sent correctly
     */
    @Test
    fun testSendStartCamera(){
        val serverThread = Thread{
            val server = Server()
            sleep(500)
            val (id, size) = server.getHeader()
            val message = server.getBody(size)
            assertEquals(MessageType.CAMERA.value, id)
            assertEquals(2, size)
            assertEquals(1, ByteBuffer.wrap(message).short)
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = VideoClient()
        client.sendStartCamera()
        client.stop()
        serverThread.join()
    }

    /**
     * Check that ReqVideoFeed message is sent correctly
     */
    @Test
    fun testSendReqVideoFeed(){
        val serverThread = Thread{
            val server = Server()
            sleep(500)
            val (id, size) = server.getHeader()
            assertEquals(MessageType.REQ_VIDEO_FEED.value, id)
            assertEquals(0, size)
            server.closeConnection()
        }
        serverThread.start()
        // Wait for the server to come online
        sleep(1000)
        val client = VideoClient()
        client.sendGetVideo()
        client.stop()
        serverThread.join()
    }
}