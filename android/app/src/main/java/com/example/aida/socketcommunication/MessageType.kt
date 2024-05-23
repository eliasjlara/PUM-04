package com.example.aida.socketcommunication

/**
 * Enum class that contains all the different
 * message types that can be sent and received
 * from the server
 */
enum class MessageType(val value: Short) {
    CAMERA(1),
    IMAGE_ANALYSIS(2),
    MIC(3),
    STT(4),
    LIDAR(5),

    REQ_VIDEO_FEED(6),
    REQ_STT(7),
    REQ_LIDAR(8),

    TEXT(9),
    VIDEO_FRAME(10),
    LIDAR_DATA(11),
    AUDIO(12),
    JOYSTICK(14); // 13 is missing, is this correct?
    // TODO ask Albin about missing number
}