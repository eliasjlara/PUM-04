import org.junit.jupiter.api.Test
import org.junit.jupiter.api.Assertions.assertEquals
import org.example.MessageType
class MessageTypeTest {
    @Test
    fun testMessageType(){
        assertEquals(1, MessageType.CAMERA.value)
        assertEquals(2, MessageType.IMAGE_ANALYSIS.value)
        assertEquals(3, MessageType.MIC.value)
        assertEquals(4, MessageType.STT.value)
        assertEquals(5, MessageType.LIDAR.value)
        assertEquals(6, MessageType.REQ_VIDEO_FEED.value)
        assertEquals(7, MessageType.REQ_STT.value)
        assertEquals(8, MessageType.REQ_LIDAR.value)
        assertEquals(9, MessageType.TEXT.value)
        assertEquals(10, MessageType.VIDEO_FRAME.value)
        assertEquals(11, MessageType.LIDAR_DATA.value)
        assertEquals(12, MessageType.AUDIO.value)
        assertEquals(14, MessageType.JOYSTICK.value)
    }
}