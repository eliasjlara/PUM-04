package com.example.aida.ui.composables

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.core.animateDpAsState
import androidx.compose.animation.core.tween
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Sensors
import androidx.compose.material.icons.filled.SensorsOff
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.Icon
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
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.aida.R
import com.example.aida.enums.ConnectionStages
import kotlinx.coroutines.delay

/**
 * Displays the lidar feed. Logic for resizing and it's animation is
 * implemented using [isLidarExpanded], [screenWidth] and [screenHeight].
 * However, the lidar data is not being fetched and therefore a placeholder
 * is in use.
 *
 * @param modifier contains UI information that needs the scope from parent
 * @param onToggleLidar check for press on lidar map. Changes the size of
 * the lidar display between small in the top right corner, and large and
 * taking up half the screen
 * @param lidarConnectionStage used to check whether the lidar is connected
 * to AIDA, if not, display connecting or error message.
 *
 * @author Elias
 */
@Composable
fun Lidar(
    modifier: Modifier,
    screenWidth: Dp,
    screenHeight: Dp,
    isLidarExpanded: Boolean,
    lidarBitmap: ImageBitmap?,
    onToggleLidar: () -> Unit,
    lidarConnectionStage: ConnectionStages
) {
    var visible by remember { mutableStateOf(true) }

    // Values used for animation
    val lidarWidth by animateDpAsState(
        targetValue = if (isLidarExpanded) screenWidth / 2 else screenWidth / 8,
        animationSpec = tween(durationMillis = 300),
        label = "animate lidar"
    )
    val lidarHeight by animateDpAsState(
        targetValue = if (isLidarExpanded) screenHeight else screenWidth / 8,
        animationSpec = tween(durationMillis = 300),
        label = "animate lidar"
    )
    val paddingSize by animateDpAsState(
        targetValue = if (isLidarExpanded) 0.dp else 10.dp,
        animationSpec = tween(durationMillis = 300),
        label = "animate padding"
    )
    val expandButtonSize = if (screenHeight / 8 < 50.dp) 20.dp else 35.dp

    Box(
        modifier = modifier
            .padding(paddingSize)
            .width(lidarWidth)
            .height(lidarHeight)
            .clickable(
                enabled = lidarConnectionStage == ConnectionStages.CONNECTION_SUCCEEDED,
                onClick = {
                    onToggleLidar()
                    visible = false
                }
            ),
    ) {
        if (lidarBitmap != null) {
            DisplayLidarFeed(lidarBitmap, visible, expandButtonSize, isLidarExpanded)
        } else {
            DisplayLidarLoadingOrErrorMessage(lidarConnectionStage)
        }
    }

    // Used to animate expand button
    LaunchedEffect(isLidarExpanded) {
        delay(300)
        visible = true
    }
}

/**
 * Displays the lidar feed
 *
 * @author Elias
 */
@Composable
fun DisplayLidarFeed(
    lidarBitmap: ImageBitmap,
    visible: Boolean,
    expandButtonSize: Dp,
    isLidarExpanded: Boolean
) {
    Box(modifier = Modifier.fillMaxSize()) {
        // Set the image to lidar feed
        Image(
            bitmap = lidarBitmap,
            contentDescription = "lidar map",
            modifier = Modifier
                .fillMaxSize()
                .border(3.dp, Color.Gray, CardDefaults.shape)
                .clip(CardDefaults.shape),
            contentScale = ContentScale.FillBounds
        )

        // Animate expand button
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
}


/**
 * Displays and loading or error message
 *
 * @author Elias
 */
@Composable
fun DisplayLidarLoadingOrErrorMessage(
    lidarConnectionStage: ConnectionStages,
) {
    var loadingText by remember { mutableStateOf("Connecting to\nlidar feed") }

    // Animate loading icon and text, i.e., rotate and add dots
    LaunchedEffect(lidarConnectionStage) {
        while (lidarConnectionStage == ConnectionStages.CONNECTING) {
            if (loadingText == "Connecting to\nlidar feed...") {
                loadingText = "Connecting to\nlidar feed"
            } else loadingText += "."
            delay(500)
        }
    }
    // Display loading or error message
    Column(
        modifier = Modifier
            .fillMaxSize()
            .border(3.dp, Color.Gray, CardDefaults.shape)
            .clip(CardDefaults.shape)
            .background(
                brush = Brush.linearGradient(
                    colors = listOf(
                        Color(0xFF2A2A2B), Color(0xFF474747), Color(0xFF5C5C5C)
                    ),
                ), alpha = 0.8f
            ),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Icon(
            imageVector = if (lidarConnectionStage == ConnectionStages.CONNECTION_FAILED)
                Icons.Filled.SensorsOff
            else
                Icons.Filled.Sensors,
            contentDescription = "Video feed icon",
            modifier = Modifier
                .size(50.dp)
                .padding(bottom = 5.dp),
            tint = Color.White,
        )

        Text(
            text = if (lidarConnectionStage == ConnectionStages.CONNECTION_FAILED)
                "Lidar feed not available"
            else
                loadingText,
            color = Color.White,
            textAlign = TextAlign.Center,
            fontSize = 14.sp,
            modifier = Modifier
                .padding(start = 6.dp, end = 6.dp)
        )
    }
}