package org.example

enum class MessageType(val value: Int) {
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