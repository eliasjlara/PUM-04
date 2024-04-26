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

/**
 * The camera page displays both the camera feed and the lidar. Logic
 * for switching between camera feed and lidar feed is currently implemented
 * but the correct data is not fetched. Therefore, placeholder pictures
 * is currently in use.
 *
 * @param screenHeight used to calculate space for page, might refactor
 * @param barHeight used to ensure that the content is padded correctly
 * @param screenWidth used to calculate space for page, might refactor
 * @author Elias
 */
@OptIn(UnstableApi::class)
@Composable
fun CameraPage(
    screenHeight: Dp,
    barHeight: Dp,
    screenWidth: Dp,
) {
    val widgetPadding = 40.dp

    Box(
        modifier = Modifier
            .padding(top = barHeight)
            .height(screenHeight - barHeight)
            .fillMaxWidth()
    ) {
        var isLidarExpanded by remember { mutableStateOf(false) }
        var isVoiceRecording by remember { mutableStateOf(false) }

        CameraFeed(isLidarExpanded = isLidarExpanded, screenWidth = screenWidth)

        Lidar(modifier = Modifier
            .align(Alignment.TopEnd)
            .zIndex(2f),
            screenWidth = screenWidth,
            screenHeight = screenHeight,
            isLidarExpanded = isLidarExpanded,
            onToggleLidar = { isLidarExpanded = !isLidarExpanded }
        )

        // Displays the text from recorded AIDA instructions, i.e. speech to text
        VoiceCommandBox(
            modifier = Modifier
                .padding(top = 3.dp, bottom = screenHeight - screenHeight / 4)
                .align(Alignment.TopCenter)
                .zIndex(10f),
            screenWidth = screenWidth,
            isVoiceRecording = isVoiceRecording,
            onToggleVoice = { isVoiceRecording = false }
        )

        RecordVoiceButton(modifier = Modifier
            .padding(bottom = widgetPadding, end = widgetPadding)
            .zIndex(3f)
            .size(120.dp)
            .clickable { isVoiceRecording = !isVoiceRecording }
            .align(Alignment.BottomEnd)
        )

        Joystick(
            Modifier
                .padding(bottom = widgetPadding, start = widgetPadding)
                .align(Alignment.BottomStart)
                .zIndex(3f),
            joystickSize = 130F,
            thumbSize = 45f,
        ) { x: Offset ->
            Log.d("JoyStick", "$x")
        }
    }
}

/**
 * Displays the camera feed. Size is switched depending on whether the
 * lidar is expanded. Placeholder picture for camera feed currently in
 * use since logic is not implemented
 *
 * @param isLidarExpanded check if lidar is expanded.
 * @author Elias
 */
@Composable
fun CameraFeed(
    screenWidth: Dp,
    isLidarExpanded: Boolean
) {
    val imageSize by animateDpAsState(
        targetValue = if (isLidarExpanded) screenWidth / 2 else screenWidth,
        animationSpec = tween(durationMillis = 300), label = "animate camera feed"
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

/**
 * Displays the lidar feed. Logic for resizing and it's animation is
 * implemented. However, the lidar data is not being fetched and therefore
 * a placeholder is in use
 *
 * @param modifier contains UI information that needs the scope from parent
 * @param isLidarExpanded check if lidar is expanded, i.e. takes up half
 * the screen
 * @param onToggleLidar check for press on lidar map. Changes the size of
 * the lidar display between small in the top right corner, and large and
 * taking up half the screen
 * @author Elias
 */
@Composable
fun Lidar(
    modifier: Modifier = Modifier,
    screenWidth: Dp,
    screenHeight: Dp,
    isLidarExpanded: Boolean,
    onToggleLidar: () -> Unit
) {
    var visible by remember { mutableStateOf(true) }

    // Values used for animation
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

/**
 * Displays the speech to text words, in a box, upon button press on
 * the RecordVoiceButton
 *
 * @param modifier contains UI information that needs the scope from parent
 * @param isVoiceRecording if voice is recording, display the box
 * @param onToggleVoice update isVoiceRecording, needed since isVoiceRecording
 * toggled both here, when the speech ends, and when the record button is pressed
 * @author Elias
 */
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

/**
 * Displays the record speech button. Used for speech to text
 *
 * @param modifier contains UI information that needs the scope from parent
 * @author Elias
 */
@Composable
fun RecordVoiceButton(modifier: Modifier = Modifier) {
    Image(
        painter = painterResource(id = R.drawable.microphone_button),
        contentDescription = "microphone",
        modifier = modifier,
    )
}

/**
 * Displays and handles logic for the joystick
 *
 * @param modifier contains UI information that needs the scope from parent
 * @param onJoystickMoved records movement from the joystick
 * @author Elias
 */
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

/**
 * Old function for video player, currently not in use.
 *
 * @author Elias
 */
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



