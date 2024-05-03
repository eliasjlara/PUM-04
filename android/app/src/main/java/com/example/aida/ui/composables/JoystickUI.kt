package com.example.aida.ui.composables

import androidx.compose.foundation.Image
import androidx.compose.foundation.gestures.detectDragGestures
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.size
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.IntOffset
import androidx.compose.ui.unit.dp
import com.example.aida.R


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
    enabled: Boolean,
    onJoystickMoved: (Offset) -> Unit = {}
) {
    val joystickCenter = joystickSize / 2
    var thumbPosition by remember { mutableStateOf(Offset(joystickCenter, joystickCenter)) }

    Box(modifier = modifier
        .size(joystickSize.dp)
        .pointerInput(Unit) {
            detectDragGestures(onDragEnd = {
                thumbPosition = Offset(joystickCenter, joystickCenter)
                onJoystickMoved(thumbPosition)
            }) { change, dragAmount ->
                if (enabled) {
                    change.consume()
                    val scaledDrag = dragAmount * 0.6f
                    val newPos = thumbPosition + scaledDrag
                    // Ensure the thumb stays within the joystick area
                    thumbPosition = Offset(
                        x = newPos.x.coerceIn(
                            (joystickSize / 5),
                            joystickSize - joystickSize / 5
                        ),
                        y = newPos.y.coerceIn(
                            (joystickSize / 5),
                            joystickSize - joystickSize / 5
                        )
                    )
                    onJoystickMoved(thumbPosition)
                }
            }
        }) {
        Image(
            painter = painterResource(id = R.drawable.joystick_outline),
            contentDescription = "Joystick background",
            modifier = Modifier.matchParentSize()
        )

        Image(painter = painterResource(id = R.drawable.joystick_thumb),
            contentDescription = "Joystick thumb",
            modifier = Modifier
                .size(thumbSize.dp)
                .offset {
                    // Calculate offset to position the thumb image correctly
                    val offsetX = (thumbPosition.x - thumbSize / 2).dp
                    val offsetY = (thumbPosition.y - thumbSize / 2).dp
                    IntOffset(offsetX.roundToPx(), offsetY.roundToPx())
                })
    }
}