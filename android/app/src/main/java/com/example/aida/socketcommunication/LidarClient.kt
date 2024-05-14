/**
 * This class is used to create a LidarClient object receive Lidar data from the server
 */
class LidarClient(ip : String = "localhost", port : Int = 12345, timeToTimeout : Int = 60000) : AbstractClient(ip, port, timeToTimeout){
    /*fun sendStartLidar(){
        val id = MessageType.LIDAR.value
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
    }*/
    /**
     * Send message to server requesting to turn on Lidar
     */
    fun sendStartLidar(){
        val id = MessageType.LIDAR.value
        val size = 2
        val start = 1 // What should this be?
        val buffer = ByteBuffer.allocate(size)
        sendRequest(id, buffer.array())

    }
    /**
     * Send request to server to start sending Lidar data
     */
    fun sentRequestLidarData(){
        val id = MessageType.REQ_LIDAR.value
        val size = 0
        val buffer = ByteBuffer.allocate(size)
        sendRequest(id, buffer.array())
    }

    fun receiveLidarData(){
        val data = fetch()
        println("Message received: ${String(message)}")
    }
}