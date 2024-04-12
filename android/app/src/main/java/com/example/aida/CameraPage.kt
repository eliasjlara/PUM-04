package com.example.aida

import android.net.Uri
import android.util.Log
import androidx.annotation.OptIn
import androidx.compose.foundation.Image
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
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
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.IntOffset
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
fun CameraPage(
    screenHeight: Dp,
    barHeight: Dp,
) {

    // Content for Camera tab
    Box(
        modifier = Modifier
            .padding(top = barHeight)
            .clip(shape)
            .height(screenHeight - barHeight)
    ) {
        // Test video
        VideoPlayerWithExoPlayer(uri = Uri.parse("http://10.0.2.2:5000/hls/playlist.m3u8"))

        Joystick(
            Modifier
                .padding(40.dp)
                .align(Alignment.BottomStart),
            joystickSize = 100F, // Size of the joystick base
            thumbSize = 30f, // Size of the joystick thumb
        ) { x: Offset ->
            Log.d("JoyStick", "$x")
        }


        Box(
            modifier = Modifier
                .padding(40.dp)
                .size(100.dp)
                .align(Alignment.BottomEnd),
            contentAlignment = Alignment.Center
        ) {
            Image(
                painter = painterResource(id = R.drawable.microphone_button),
                contentDescription = "microphone",
                modifier = Modifier
                    .size(100.dp)
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
    val joystickCenter = joystickSize / 2
    var thumbPosition by remember { mutableStateOf(Offset(joystickCenter, joystickCenter)) }

    Box(
        modifier = modifier
            .size(joystickSize.dp)
            .pointerInput(Unit) {
                detectDragGestures(onDragEnd = {
                    thumbPosition = Offset(joystickCenter, joystickCenter)
                    onJoystickMoved(thumbPosition)
                }) { change, dragAmount ->
                    change.consume()
                    val scaledDrag = dragAmount * 0.5f
                    val newPos = thumbPosition + scaledDrag
                    // Ensure the thumb stays within the joystick area
                    thumbPosition = Offset(
                        x = newPos.x.coerceIn((joystickSize / 5), joystickSize - joystickSize / 5),
                        y = newPos.y.coerceIn((joystickSize / 5), joystickSize - joystickSize / 5)
                    )
                    onJoystickMoved(thumbPosition)
                }
            }
    ) {
        // Background joystick image
        Image(
            painter = painterResource(id = R.drawable.joystick_outline),
            contentDescription = "Joystick background",
            modifier = Modifier.matchParentSize()
        )

        // Thumb image, positioned according to thumbPosition
        Image(
            painter = painterResource(id = R.drawable.joystick_thumb),
            contentDescription = "Joystick thumb",
            modifier = Modifier
                .size(thumbSize.dp) // Set the size of the thumb image
                .offset {
                    // Calculate offset to position the thumb image correctly
                    val offsetX = (thumbPosition.x - thumbSize / 2).dp
                    val offsetY = (thumbPosition.y - thumbSize / 2).dp
                    IntOffset(offsetX.roundToPx(), offsetY.roundToPx())
                }
        )
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
