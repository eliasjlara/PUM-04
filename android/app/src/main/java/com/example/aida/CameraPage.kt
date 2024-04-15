package com.example.aida

import android.net.Uri
import androidx.annotation.OptIn
import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.core.animateDpAsState
import androidx.compose.animation.core.tween
import androidx.compose.animation.expandVertically
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.animation.shrinkVertically
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.CardDefaults.shape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.compose.ui.zIndex
import androidx.media3.common.MediaItem
import androidx.media3.common.PlaybackException
import androidx.media3.common.Player
import androidx.media3.common.util.Log
import androidx.media3.common.util.UnstableApi
import androidx.media3.exoplayer.ExoPlayer
import androidx.media3.exoplayer.trackselection.DefaultTrackSelector
import androidx.media3.ui.AspectRatioFrameLayout
import androidx.media3.ui.PlayerView
import kotlinx.coroutines.delay

@OptIn(UnstableApi::class)
@Composable
fun CameraPage(
    screenHeight: Dp,
    barHeight: Dp,
    screenWidth: Dp,
) {
    Box(
        modifier = Modifier
            .padding(top = barHeight)
            .height(screenHeight - barHeight)
            .fillMaxWidth()
    ) {
        var isLidarExpanded by remember { mutableStateOf(false) }
        var isVoiceRecording by remember { mutableStateOf(false) }

        Background(isLidarExpanded = isLidarExpanded, screenWidth = screenWidth)

        Lidar(modifier = Modifier
            .align(Alignment.TopEnd)
            .zIndex(2f),
            screenWidth = screenWidth,
            screenHeight = screenHeight,
            isLidarExpanded = isLidarExpanded,
            onToggleLidar = { isLidarExpanded = !isLidarExpanded }
        )

        VoiceCommandBox(
            modifier = Modifier
                .padding(top = 3.dp, bottom = screenHeight - screenHeight / 4)
                .align(Alignment.TopCenter)
                .zIndex(10f),
            screenWidth = screenWidth,
            isVoiceRecording = isVoiceRecording,
            onToggleVoice = { isVoiceRecording = false }
        )

        //VideoPlayerWithExoPlayer(uri = Uri.parse("http://10.0.2.2:5000/hls/playlist.m3u8"))

        RecordVoiceButton(modifier = Modifier
            .padding(bottom = 40.dp, end = 40.dp)
            .zIndex(3f)
            .size(120.dp)
            .clickable { isVoiceRecording = !isVoiceRecording }
            .align(Alignment.BottomEnd)
        )

        Joystick(
            Modifier
                .padding(bottom = 40.dp, start = 40.dp)
                .align(Alignment.BottomStart)
                .zIndex(3f),
            joystickSize = 130F,
            thumbSize = 45f,
        ) { x: Offset ->
            Log.d("JoyStick", "$x")
        }
    }
}

@Composable
fun VoiceCommandBox(
    modifier: Modifier = Modifier,
    screenWidth: Dp,
    isVoiceRecording: Boolean,
    onToggleVoice: () -> Unit
) {
    var voiceCommandString by remember { mutableStateOf("I am listening ...") }

    AnimatedVisibility(
        visible = isVoiceRecording,
        enter = expandVertically(),
        exit = shrinkVertically(),
        modifier = modifier
    ) {
        Box(
            modifier = Modifier
                .background(
                    Color(0x86000000),
                    shape = RoundedCornerShape(
                        topStart = 0.dp,
                        topEnd = 0.dp,
                        bottomStart = 20.dp,
                        bottomEnd = 20.dp
                    )
                )
                .width(screenWidth / 2)
                .fillMaxHeight(0.5f),
            Alignment.Center
        ) {
            Text(text = voiceCommandString, color = Color.White, fontSize = 18.sp)
        }
    }

    LaunchedEffect(isVoiceRecording) {
        if (isVoiceRecording) {
            voiceCommandString = "I am listening ..."
            delay(3000)
            voiceCommandString = "\"Drive\""
            delay(1000)
            voiceCommandString = "\"Drive 3 meters\""
            delay(700)
            voiceCommandString = "\"Drive 3 meters forward\""
            delay(5000)
            voiceCommandString = "Carrying out the action in sequence"
            delay(5000)
            onToggleVoice()
        }
    }
}

@Composable
fun Background(
    screenWidth: Dp,
    isLidarExpanded: Boolean
) {
    val imageSize by animateDpAsState(
        targetValue = if (isLidarExpanded) screenWidth / 2 else screenWidth,
        animationSpec = tween(durationMillis = 300), label = "animate lidar"
    )

    Image(
        painter = painterResource(id = R.drawable.aida_preview_image),
        contentDescription = "aida preview image",
        modifier = Modifier
            .width(imageSize)
            .fillMaxHeight()
            .border(3.dp, Color.Gray, shape)
            .clip(shape)
            .zIndex(1f),
        contentScale = ContentScale.Crop
    )
}

@Composable
fun Lidar(
    modifier: Modifier = Modifier,
    screenWidth: Dp,
    screenHeight: Dp,
    isLidarExpanded: Boolean,
    onToggleLidar: () -> Unit
) {
    var visible by remember { mutableStateOf(true) }

    val lidarWidth by animateDpAsState(
        targetValue = if (isLidarExpanded) screenWidth / 2 else screenWidth / 8,
        animationSpec = tween(durationMillis = 300), label = "animate lidar"
    )
    val lidarHeight by animateDpAsState(
        targetValue = if (isLidarExpanded) screenHeight else screenWidth / 8,
        animationSpec = tween(durationMillis = 300), label = "animate lidar"
    )
    val paddingSize by animateDpAsState(
        targetValue = if (isLidarExpanded) 0.dp else 10.dp,
        animationSpec = tween(durationMillis = 300), label = "animate padding"
    )

    val expandButtonSize = if (screenHeight / 8 < 50.dp) 20.dp else 35.dp

    Box(
        modifier = modifier
            .padding(paddingSize)
            .width(lidarWidth)
            .height(lidarHeight)
            .clickable {
                onToggleLidar()
                visible = false
            },
    ) {
        Image(
            painter = painterResource(id = R.drawable.lidar_small),
            contentDescription = "lidar map",
            modifier = Modifier
                .fillMaxSize()
                .border(3.dp, Color.Gray, shape)
                .clip(shape),
            contentScale = ContentScale.FillBounds
        )
        AnimatedVisibility(
            visible = visible,
            enter = fadeIn(),
            exit = fadeOut(),
            modifier = Modifier
                .padding(2.dp)
                .size(expandButtonSize)
                .align(if (isLidarExpanded) Alignment.TopEnd else Alignment.BottomStart)
                .rotate(if (isLidarExpanded) 180f else 0f)
                .alpha(if (visible) 1f else 0f),
        ) {
            Image(
                painter = painterResource(id = R.drawable.lidar_expand),
                contentDescription = "lidar expand",
            )
        }
    }
    LaunchedEffect(isLidarExpanded) {
        delay(300)
        visible = true
    }
}

@Composable
fun RecordVoiceButton(modifier: Modifier = Modifier) {
    Image(
        painter = painterResource(id = R.drawable.microphone_button),
        contentDescription = "microphone",
        modifier = modifier,
    )
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
                    val scaledDrag = dragAmount * 0.6f
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
        Image(
            painter = painterResource(id = R.drawable.joystick_outline),
            contentDescription = "Joystick background",
            modifier = Modifier.matchParentSize()
        )

        Image(
            painter = painterResource(id = R.drawable.joystick_thumb),
            contentDescription = "Joystick thumb",
            modifier = Modifier
                .size(thumbSize.dp)
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
            Text(
                "Can't connect to server, try restarting the app",
                Modifier.align(Alignment.Center)
            )
        }
    }
}



