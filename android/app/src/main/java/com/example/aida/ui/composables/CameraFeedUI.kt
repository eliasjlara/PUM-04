package com.example.aida.ui.composables

import androidx.compose.animation.core.LinearEasing
import androidx.compose.animation.core.animateDpAsState
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.HourglassTop
import androidx.compose.material.icons.filled.VideocamOff
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.graphicsLayer
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.zIndex
import com.example.aida.enums.ConnectionStages
import kotlinx.coroutines.delay

/**
 * Displays the camera feed. Size is switched depending on whether the
 * [lidarIsExpanded] using the [screenWidth] of the device. The video
 * feed is a [imageBitmap] that is continuously fetched from AIDA.
 *
 * @param cameraFeedConnectionStage used to determine whether to show the
 * feed
 * @param ipAddress used to display error message
 * @param port used to display error message
 *
 * @author Elias
 */
@Composable
fun CameraFeed(
    lidarIsExpanded: Boolean,
    screenWidth: Dp,
    imageBitmap: ImageBitmap?,
    cameraFeedConnectionStage: ConnectionStages,
    ipAddress: String,
    port: Int
) {
    // Animation for resizing camera feedA
    val imageSize by animateDpAsState(
        targetValue = if (lidarIsExpanded) screenWidth / 2 else screenWidth,
        animationSpec = tween(durationMillis = 300),
        label = "animate camera feed"
    )

    val standardModifier = Modifier
        .width(imageSize)
        .fillMaxHeight()
        .zIndex(1f)

    if (imageBitmap != null) {
        DisplayCameraFeed(
            imageBitmap = imageBitmap,
            modifier = standardModifier
        )
    } else {
        DisplayLoadingOrErrorMessage(
            cameraFeedConnectionStage = cameraFeedConnectionStage,
            ipAddress = ipAddress,
            port = port,
            modifier = standardModifier
        )
    }
}

/**
 * Displays the camera feed using the [imageBitmap]

 * @param modifier used to structure UI
 *
 * @author Elias
 */
@Composable
fun DisplayCameraFeed(
    imageBitmap: ImageBitmap,
    modifier: Modifier
) {
    // Set the feed image to the imageBitmap
    Image(
        bitmap = imageBitmap,
        contentDescription = "aida preview image",
        modifier = modifier
            .border(3.dp, Color.Gray, CardDefaults.shape)
            .clip(CardDefaults.shape),
        contentScale = ContentScale.Crop
    )
}


/**
 * Displays and loading or error message using the
 * [cameraFeedConnectionStage]. The error message
 * contains the [ipAddress] and [port] for ease of
 * use.

 * @param modifier used to structure UI
 *
 * @author Elias
 */
@Composable
fun DisplayLoadingOrErrorMessage(
    cameraFeedConnectionStage: ConnectionStages,
    ipAddress: String,
    port: Int,
    modifier: Modifier
) {
    // Display loading or error message
    Column(
        modifier = modifier
            .background(
                brush = Brush.linearGradient( // Apply fancy color gradient
                    colors = listOf(
                        Color(0xFF0D1C22), Color(0xFF152830), Color(0xFF5D69A5)
                    ),
                    start = Offset(0f, Float.POSITIVE_INFINITY),
                    end = Offset(Float.POSITIVE_INFINITY, 0f)
                ), alpha = 0.8f
            ),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        var loadingText by remember { mutableStateOf("Connecting to camera feed") }
        var targetRotationDegrees by remember { mutableFloatStateOf(0f) }
        // Degrees used when rotating loading icon
        val rotationDegrees = animateFloatAsState(
            targetValue = targetRotationDegrees, animationSpec = tween(
                durationMillis = 600, // Duration for the rotation to complete 180 degrees
                easing = LinearEasing
            ), label = ""
        )

        // Animate loading icon and text, i.e., rotate and add dots
        LaunchedEffect(cameraFeedConnectionStage) {
            while (cameraFeedConnectionStage == ConnectionStages.CONNECTING) {
                if (loadingText == "Connecting to camera feed...") {
                    loadingText = "Connecting to camera feed"
                    targetRotationDegrees += 180f
                } else
                    loadingText += "."
                delay(1000)
            }
        }

        // Display appropriate icon based on connection stage
        Icon(
            imageVector = if (cameraFeedConnectionStage == ConnectionStages.CONNECTING)
                Icons.Filled.HourglassTop
            else
                Icons.Filled.VideocamOff,
            contentDescription = "Video feed icon",
            modifier = Modifier
                .size(60.dp)
                .graphicsLayer(
                    rotationZ = if (cameraFeedConnectionStage == ConnectionStages.CONNECTING)
                        rotationDegrees.value
                    else
                        0f
                ),
            tint = Color.LightGray
        )

        // Display loading/error message text
        Text(
            text = when (cameraFeedConnectionStage) {
                ConnectionStages.CONNECTING -> loadingText
                ConnectionStages.CONNECTION_FAILED -> "Camera feed not available"
                ConnectionStages.CONNECTION_CLOSED -> "Camera feed is off"
                else -> ""
            },
            textAlign = TextAlign.Center,
            fontWeight = FontWeight.Bold,
            style = MaterialTheme.typography.headlineSmall,
            color = Color.LightGray,
            modifier = Modifier.padding(5.dp)
        )

        Text(
            text = when (cameraFeedConnectionStage) {
                ConnectionStages.CONNECTING -> "The camera feed is currently fetching from AIDA\n" +
                        "Please wait until a connection is made"
                ConnectionStages.CONNECTION_FAILED -> "Could not connect to AIDA, please try again\n" +
                        "You are trying to connect to: $ipAddress:$port"
                ConnectionStages.CONNECTION_CLOSED -> "Please press button"
                else -> ""
            },
            textAlign = TextAlign.Center,
            color = Color.LightGray
        )
    }
}