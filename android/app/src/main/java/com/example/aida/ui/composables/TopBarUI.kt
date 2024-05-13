package com.example.aida.ui.composables

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.scale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.aida.R
import com.example.aida.ui.theme.TopBarColor

/**
 * Function for the top bar, contains both UI and logic, however some
 * widgets need to be decided on what they should do.
 *
 * @param onMenuClicked records if the menu button has been pressed
 * @param onCameraClicked records if the camera on button has been pressed
 * @param barHeight used to determine the size of the top bar
 * @param topBarTitle used when want to dynamically change the title
 * @author Elias
 */
@Composable
fun TopBar(
    onMenuClicked: () -> Unit,
    onCameraClicked: () -> Unit,
    barHeight: Dp,
    topBarTitle: String,
) {
    val barPadding = 15.dp

    Row(
        modifier = Modifier
            .height(barHeight)
            .fillMaxWidth()
            .background(TopBarColor.copy(alpha = 0.6f))
            .padding(start = barPadding, end = barPadding),
        horizontalArrangement = Arrangement.Absolute.SpaceEvenly,
        verticalAlignment = Alignment.CenterVertically
    ) {
        // Set the menu icon
        Row(
            modifier = Modifier
                .weight(1f)
        ) {
            Image(
                painter = painterResource(id = R.drawable.configuration_button),
                contentDescription = "configuration",
                Modifier
                    .clickable(onClick = onMenuClicked)
                    .scale(1.2f)
            )
        }

        // Set text in the middle of the top bar
        Row(
            modifier = Modifier.weight(1f),
            horizontalArrangement = Arrangement.Center
        ) {
            Text(
                text = topBarTitle,
                fontSize = 16.sp
            )
        }

        // Set camera and volume on/off buttons
        Row(
            modifier = Modifier.weight(1f),
            horizontalArrangement = Arrangement.Center,
            verticalAlignment = Alignment.CenterVertically
        ) {
            var cameraPress by remember { mutableStateOf("on") }
            var speakerPress by remember { mutableStateOf("on") }

            Spacer(Modifier.weight(5f))

            Column(
                modifier = Modifier
                    .clickable(onClick = {
                        cameraPress = if (cameraPress == "on") "off" else "on"
                        onCameraClicked()
                    }
                    )
                    .padding(top = 10.dp)
            ) {
                Image(
                    painter = painterResource(id = R.drawable.camera_button),
                    contentDescription = "camera",
                    Modifier
                        .scale(1.2f)
                )
                Text(
                    text = cameraPress,
                    Modifier
                        .offset(y = (-2).dp)
                )
            }
            Spacer(Modifier.weight(1f))

            Column(
                horizontalAlignment = Alignment.CenterHorizontally,
                modifier = Modifier
                    .clickable(onClick = {
                        speakerPress = if (speakerPress == "on") "off" else "on"

                    }
                    )
                    .padding(top = 10.dp)
            ) {
                Image(
                    painter = painterResource(id = R.drawable.volume_button),
                    contentDescription = "camera",
                    Modifier
                        .scale(1.2f)
                )
                Text(
                    text = speakerPress,
                    Modifier
                        .offset(y = (-2).dp)
                )
            }
            Spacer(Modifier.weight(1f))
            Text(
                text = "Battery: " + 69 + "%",
                fontSize = 16.sp
            )
            Spacer(Modifier.weight(1f))

            Text(
                text = "Lag: " + 12 + "ms",
                fontSize = 16.sp
            )
        }
    }
}