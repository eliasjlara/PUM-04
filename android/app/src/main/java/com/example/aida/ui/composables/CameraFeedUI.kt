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
 * lidar is expanded. Placeholder picture for camera feed currently in
 * use since logic is not implemented
 *
 * @param isLidarExpanded check if lidar is expanded.
 * @author Elias
 */
@Composable
fun CameraFeed(
    screenWidth: Dp,
    isLidarExpanded: Boolean,
    imageBitmap: ImageBitmap?,
    cameraFeedConnectionStage: ConnectionStages,
    ipAddress: String,
    port: Int
) {
    val imageSize by animateDpAsState(
        targetValue = if (isLidarExpanded) screenWidth / 2 else screenWidth,
        animationSpec = tween(durationMillis = 300),
        label = "animate camera feed"
    )
    if (imageBitmap != null) {
        Image(
            bitmap = imageBitmap,
            contentDescription = "aida preview image",
            modifier = Modifier
                .width(imageSize)
                .fillMaxHeight()
                .border(3.dp, Color.Gray, CardDefaults.shape)
                .clip(CardDefaults.shape)
                .zIndex(1f),
            contentScale = ContentScale.Crop
        )
    } else {
        Column(
            modifier = Modifier
                .width(imageSize)
                .fillMaxHeight()
                .zIndex(1f)
                .background(
                    brush = Brush.linearGradient(
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
            val rotationDegrees = animateFloatAsState(
                targetValue = targetRotationDegrees, animationSpec = tween(
                    durationMillis = 600, // Duration for the rotation to complete 180 degrees
                    easing = LinearEasing
                ), label = ""
            )

            LaunchedEffect(cameraFeedConnectionStage) {
                while (cameraFeedConnectionStage == ConnectionStages.CONNECTING) {
                    if (loadingText == "Connecting to camera feed...") {
                        loadingText = "Connecting to camera feed"
                        targetRotationDegrees += 180f
                    } else loadingText += "."
                    delay(1000)
                }
            }
            Icon(
                imageVector = if (cameraFeedConnectionStage == ConnectionStages.CONNECTING) Icons.Filled.HourglassTop else Icons.Filled.VideocamOff,
                contentDescription = "Video feed icon",
                modifier = Modifier
                    .size(60.dp)
                    .graphicsLayer(rotationZ = rotationDegrees.value),
                tint = Color.LightGray
            )
            Text(
                text = if (cameraFeedConnectionStage == ConnectionStages.CONNECTING) loadingText else "Camera feed not available",
                textAlign = TextAlign.Center,
                fontWeight = FontWeight.Bold,
                style = MaterialTheme.typography.headlineSmall,
                color = Color.LightGray,
                modifier = Modifier.padding(5.dp)
            )
            Text(
                text = (if (cameraFeedConnectionStage == ConnectionStages.CONNECTING)
                    "The camera feed is currently fetching from AIDA\n" +
                            "Please wait until a connection is made"
                else
                    "Could not connect to AIDA, please try again\n" +
                            "You are trying to connect to: $ipAddress:$port"),
                textAlign = TextAlign.Center,
                color = Color.LightGray
            )
        }
    }
}