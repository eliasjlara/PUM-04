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
import com.example.aida.enums.ConnectionStages
import kotlinx.coroutines.flow.StateFlow

/**
 * Displays the joystick based on [joystickSize] and [thumbSize] and if
 * the joystick is connetected to AIDA, i.e, [enabled]. The logic for
 * the joystick is used in [onJoystickMoved] and sent to AIDA.
 *
 * @param modifier contains UI information that needs the scope from parent
 *
 * @author Elias
 */
@Composable
fun Joystick(
    modifier: Modifier,
    joystickSize: Float,
    thumbSize: Float,
    enabled: StateFlow<ConnectionStages>,
    onJoystickMoved: (Offset) -> Unit = {}
) {
    val joystickCenter = joystickSize / 2
    var thumbPosition by remember { mutableStateOf(Offset(joystickCenter, joystickCenter)) }
    val minPixelValue = joystickSize / 5
    val maxPixelValue = joystickSize - joystickSize / 5
    Box(modifier = modifier
        .size(joystickSize.dp)
        .pointerInput(Unit) {
            detectDragGestures(onDragEnd = {
                thumbPosition = Offset(joystickCenter, joystickCenter)
                val normalizedPositionX = normalizePosition(joystickCenter, minPixelValue, maxPixelValue)
                val normalizedPositionY = normalizePosition(joystickCenter, minPixelValue, maxPixelValue)
                val normalizedPosition = Offset(normalizedPositionX, normalizedPositionY)
                onJoystickMoved(normalizedPosition)
            }) { change, dragAmount ->
                if (enabled.value == ConnectionStages.CONNECTION_SUCCEEDED) {
                    change.consume()
                    val scaledDrag = dragAmount * 0.6f
                    val newPos = thumbPosition + scaledDrag
                    // Ensure the thumb stays within the joystick area
                    thumbPosition = Offset(
                        x = newPos.x.coerceIn(
                            minPixelValue,
                            maxPixelValue
                        ),
                        y = newPos.y.coerceIn(
                            minPixelValue,
                            maxPixelValue
                        )
                    )
                    val normalizedPositionX = normalizePosition(thumbPosition.x, minPixelValue, maxPixelValue)
                    val normalizedPositionY = normalizePosition(thumbPosition.y, minPixelValue, maxPixelValue)
                    val normalizedPosition = Offset(normalizedPositionX, normalizedPositionY)
                    onJoystickMoved(normalizedPosition)
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

/**
 * Normalize the position of between the values between -1 and 1
 */
private fun normalizePosition(value: Float, lowerValue: Float, upperValue: Float): Float {
    return 2.0f * (value - lowerValue) / (upperValue - lowerValue) - 1.0f
}