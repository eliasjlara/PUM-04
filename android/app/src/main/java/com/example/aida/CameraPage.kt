package com.example.aida

import android.net.Uri
import android.util.Log
import androidx.annotation.OptIn
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.CardDefaults.shape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.media3.common.MediaItem
import androidx.media3.common.PlaybackException
import androidx.media3.common.Player
import androidx.media3.common.util.UnstableApi
import androidx.media3.exoplayer.ExoPlayer
import androidx.media3.exoplayer.trackselection.DefaultTrackSelector
import androidx.media3.ui.AspectRatioFrameLayout
import androidx.media3.ui.PlayerView

@Composable
fun CameraPage() {

    // Content for Camera tab
    Box(
        modifier = Modifier
            .clip(shape)
            .fillMaxSize()
    ) {
        // Test video
        VideoPlayerWithExoPlayer(uri = Uri.parse("http://10.0.2.2:5000/hls/playlist.m3u8"))

        Joystick(
            Modifier
                .padding(40.dp)
                .align(Alignment.BottomStart)
                .clip(shape)
                .border(BorderStroke(3.dp, Color.Black))
                .background(Color.DarkGray.copy(0.6F))
                .alpha(0.6F),
            joystickSize = 100F, // Size of the joystick base
            thumbSize = 45f, // Size of the joystick thumb
        ) { x: Offset ->
            Log.d("JoyStick", "$x")
        }

        Box(
            modifier = Modifier
                .padding(40.dp)
                .clip(shape)
                .border(BorderStroke(3.dp, Color.Black))
                .background(Color.DarkGray.copy(0.6F))
                .alpha(0.6F)
                .size(100.dp)
                .align(Alignment.BottomEnd),
            contentAlignment = Alignment.Center
        ) {
            Text(
                "Widget 2",
                color = Color.White
            )
        }
    }
}

@Composable
fun Joystick(
    modifier: Modifier = Modifier,
    joystickSize: Float,
    thumbSize: Float,
    onJoystickMoved: (Offset) -> Unit = {}
) {
    var thumbPosition by remember { mutableStateOf(Offset(joystickSize / 2, joystickSize / 2)) }

    Box(
        modifier = modifier
            .size(joystickSize.dp)
            .pointerInput(Unit) {
                detectDragGestures(onDragEnd = {
                    thumbPosition = Offset(joystickSize / 2, joystickSize / 2)
                    onJoystickMoved(thumbPosition)
                }) { change, dragAmount ->
                    change.consume()
                    val newPos = thumbPosition + dragAmount
                    // Ensure the thumb stays within the joystick area
                    thumbPosition = Offset(
                        x = newPos.x.coerceIn(0f, joystickSize),
                        y = newPos.y.coerceIn(0f, joystickSize)
                    )
                    onJoystickMoved(thumbPosition)
                }
            }
    ) {
        Canvas(modifier = Modifier.matchParentSize()) {
            // Calculate the center of the canvas
            val centerX = size.width / 2
            val centerY = size.height / 2

            // Joystick
            drawCircle(
                color = Color.LightGray,
                center = Offset(centerX, centerY),
                radius = joystickSize // Assuming joystickSize is the diameter
            )

            // Adjust thumbPosition to be relative to the canvas center
            val thumbCenterX = centerX + (thumbPosition.x - joystickSize / 2)
            val thumbCenterY = centerY + (thumbPosition.y - joystickSize / 2)

            // Joystick thumb
            drawCircle(
                color = Color.DarkGray,
                center = Offset(thumbCenterX, thumbCenterY),
                radius = thumbSize // Assuming thumbSize is the diameter
            )
        }
    }
}

@OptIn(UnstableApi::class)
@Composable
fun VideoPlayerWithExoPlayer(uri: Uri) {

    val context = LocalContext.current
    val showError = remember { mutableStateOf(false) }

    val trackSelector = DefaultTrackSelector(context).apply {
        // Need for now so that it doesn't change the resolution automatically
        parameters = buildUponParameters().setMaxVideoSizeSd().build()
    }
    val mediaItem = MediaItem.Builder()
        .setUri(uri)
        .setLiveConfiguration(
            MediaItem.LiveConfiguration.Builder()
                .setMaxPlaybackSpeed(1.02f) // Allow slight speed variation for live adjustment
                .build()
        )
        .build()

    val exoPlayer = ExoPlayer.Builder(context)
        .setTrackSelector(trackSelector)
        .build().also { player ->
            player.setMediaItem(mediaItem)
            player.prepare()
            player.playWhenReady = true
        }

    // Listener to retry connection to server
    // I could not get this to work :((
    exoPlayer.addListener(object : Player.Listener {
        override fun onPlayerError(error: PlaybackException) {
            super.onPlayerError(error)

            showError.value = true
        }
    })

    if (!showError.value) {
        AndroidView(
            factory = { ctx ->
                PlayerView(ctx).apply {
                    resizeMode = AspectRatioFrameLayout.RESIZE_MODE_ZOOM
                    player = exoPlayer
                    useController = false
                }
            },
            modifier = Modifier.fillMaxSize(),
            update = { view ->
                view.player = exoPlayer
            }
        )
    } else {
        Box(modifier = Modifier.fillMaxSize()) {
            Text("Can't connect to server, try restarting the app", Modifier.align(Alignment.Center))
        }
    }
}
