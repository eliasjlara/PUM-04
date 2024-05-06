package com.example.aida.ui.composables

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.expandVertically
import androidx.compose.animation.shrinkVertically
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.aida.R

/**
 * Displays the speech in a text format using [voiceCommandString], in a
 * box, upon button press on the RecordVoiceButton
 *
 * @param modifier contains UI information that needs the scope from parent
 * @param screenWidth used to set the size of the box
 * @param isVoiceRecording if voice is recording, display the box. This is
 * toggled both here, when the speech ends, and when the record button is
 * pressed
 *
 * @author Elias
 */
@Composable
fun VoiceCommandBox(
    modifier: Modifier,
    screenWidth: Dp,
    isVoiceRecording: Boolean,
    voiceCommandString: String
) {
    AnimatedVisibility(
        visible = isVoiceRecording,
        enter = expandVertically(),
        exit = shrinkVertically(),
        modifier = modifier
    ) {
        Box(
            modifier = Modifier
                .background(
                    Color(0x86000000), shape = RoundedCornerShape(
                        topStart = 0.dp, topEnd = 0.dp, bottomStart = 20.dp, bottomEnd = 20.dp
                    )
                )
                .width(screenWidth / 2)
                .fillMaxHeight(0.5f), Alignment.Center
        ) {
            Text(text = voiceCommandString, color = Color.White, fontSize = 18.sp)
        }
    }
}

/**
 * Displays the record speech button. Used for speech to text
 *
 * @param modifier contains UI information that needs the scope from parent
 *
 * @author Elias
 */
@Composable
fun RecordVoiceButton(
    modifier: Modifier
) {
    Image(
        painter = painterResource(id = R.drawable.microphone_button),
        contentDescription = "microphone",
        modifier = modifier
    )
}
