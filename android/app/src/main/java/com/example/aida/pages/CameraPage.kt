package com.example.aida.pages

import androidx.annotation.OptIn
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.livedata.observeAsState
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.zIndex
import androidx.media3.common.util.Log
import androidx.media3.common.util.UnstableApi
import com.example.aida.enums.ConnectionStages
import com.example.aida.ui.composables.CameraFeed
import com.example.aida.ui.composables.Joystick
import com.example.aida.ui.composables.Lidar
import com.example.aida.ui.composables.RecordVoiceButton
import com.example.aida.ui.composables.VoiceCommandBox
import com.example.aida.viewmodels.MainViewModel

/**
 * The camera page displays both the camera feed and the lidar. Logic
 * for switching between camera feed and lidar feed is currently implemented
 * but the correct data is not fetched. Therefore, placeholder pictures
 * is currently in use.
 *
 * @param screenHeight used to calculate space for page, might refactor
 * @param barHeight used to ensure that the content is padded correctly
 * @param screenWidth used to calculate space for page, might refactor
 * @param viewModel contains connection information, used to check whether
 * each connection is made, as well as fetching the image and lidar. Also
 * handles the speech to text initiation and fetching.
 * @author Elias
 */
@OptIn(UnstableApi::class)
@Composable
fun CameraPage(
    screenHeight: Dp,
    barHeight: Dp,
    screenWidth: Dp,
    viewModel: MainViewModel,
) {
    val widgetPadding = 40.dp

    Box(
        modifier = Modifier
            .padding(top = barHeight)
            .height(screenHeight - barHeight)
            .fillMaxWidth()
    ) {
        var isLidarExpanded by remember { mutableStateOf(false) }
        val isVoiceRecording by viewModel.isRecording.observeAsState(initial = false)

        CameraFeed(
            isLidarExpanded = isLidarExpanded,
            screenWidth = screenWidth,
            imageBitmap = viewModel.imageBitmap.collectAsState().value,
            cameraFeedConnectionStage = viewModel.cameraFeedConnectionStage.collectAsState().value,
            ipAddress = viewModel.ipAddress.collectAsState().value,
            port = viewModel.port.collectAsState().value,
        )

        Lidar(
            modifier = Modifier
                .align(Alignment.TopEnd)
                .zIndex(2f),
            screenWidth = screenWidth,
            screenHeight = screenHeight,
            isLidarExpanded = isLidarExpanded,
            onToggleLidar = { isLidarExpanded = !isLidarExpanded },
            lidarConnectionStage = viewModel.lidarConnectionStage.collectAsState().value,
        )
        val voiceCommand by viewModel.voiceCommand.observeAsState("I am listening ...")

        // Displays the text from recorded AIDA instructions, i.e. speech to text
        VoiceCommandBox(
            modifier = Modifier
                .padding(top = 3.dp, bottom = screenHeight - screenHeight / 4)
                .align(Alignment.TopCenter)
                .zIndex(10f),
            screenWidth = screenWidth,
            isVoiceRecording = isVoiceRecording,
            voiceCommandString = voiceCommand
        )
        // TODO: Change sttConnection to Joystick

        val sttConnected =
            (viewModel.sttConnectionStage.collectAsState().value == ConnectionStages.CONNECTION_SUCCEEDED)

        Joystick(
            Modifier
                .padding(bottom = widgetPadding, start = widgetPadding)
                .align(Alignment.BottomStart)
                .zIndex(3f)
                .alpha(if (sttConnected) 1.0f else 0.3f),
            joystickSize = 130F,
            thumbSize = 45f,
            enabled = sttConnected
        ) { x: Offset ->
            Log.d("JoyStick", "$x")
        }
        RecordVoiceButton(
            modifier = Modifier
                .padding(bottom = widgetPadding, end = widgetPadding)
                .zIndex(3f)
                .size(120.dp)
                .align(Alignment.BottomEnd)
                .alpha(if (sttConnected) 1.0f else 0.3f)
                .clip(CircleShape)
                .clickable(
                    enabled = sttConnected,
                    onClick = {
                        viewModel.startVoiceRecording()
                    })
        )
    }
}
