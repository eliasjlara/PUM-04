import com.example.aida.socketcommunication.MessageType
import org.junit.Assert.assertEquals
import org.junit.Test

class MessageTypeTest {
    @Test
    fun testCameraValue(){
        val expected = 1.toShort()
        assertEquals(expected, MessageType.CAMERA.value)
    }
    @Test
    fun testImageAnalysisValue(){
        val expected = 2.toShort()
        assertEquals(expected, MessageType.IMAGE_ANALYSIS.value)
    }
    @Test
    fun testMicValue(){
        val expected = 3.toShort()
        assertEquals(expected, MessageType.MIC.value)
    }
    @Test
    fun testSttValue(){
        val expected = 4.toShort()
        assertEquals(expected, MessageType.STT.value)
    }
    @Test
    fun testLidarValue(){
        val expected = 5.toShort()
        assertEquals(expected, MessageType.LIDAR.value)
    }
    @Test
    fun testReqVideoFeedValue(){
        val expected = 6.toShort()
        assertEquals(expected, MessageType.REQ_VIDEO_FEED.value)
    }
    @Test
    fun testReqSttValue(){
        val expected = 7.toShort()
        assertEquals(expected, MessageType.REQ_STT.value)
    }
    @Test
    fun testReqLidarValue(){
        val expected = 8.toShort()
        assertEquals(expected, MessageType.REQ_LIDAR.value)
    }
    @Test
    fun testTextValue(){
        val expected = 9.toShort()
        assertEquals(expected, MessageType.TEXT.value)
    }
    @Test
    fun testVideoFrameValue(){
        val expected = 10.toShort()
        assertEquals(expected, MessageType.VIDEO_FRAME.value)
    }
    @Test
    fun testLidarDataValue(){
        val expected = 11.toShort()
        assertEquals(expected, MessageType.LIDAR_DATA.value)
    }
    @Test
    fun testAudioValue(){
        val expected = 12.toShort()
        assertEquals(expected, MessageType.AUDIO.value)
    }
    @Test
    fun testJoystickValue(){
        val expected = 14.toShort()
        assertEquals(expected, MessageType.JOYSTICK.value)
    }
}