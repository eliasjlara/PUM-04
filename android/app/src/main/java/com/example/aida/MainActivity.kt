package com.example.aida

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.layout.wrapContentHeight
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.zIndex
import com.example.aida.ui.theme.AIDATheme


class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            AIDATheme {
                // A surface container using the 'background' color from the theme
                Surface(
                    color = MaterialTheme.colorScheme.background
                ) {
                    val configuration = LocalConfiguration.current

                    val screenHeight = configuration.screenHeightDp.dp
                    val screenWidth = configuration.screenWidthDp.dp

                    val barHeight = screenHeight / 8

                    var state by remember { mutableIntStateOf(0) }

                    // Segmented Buttons Row, handles the change of tabs


                    TopBar(
                        onConfigurationClicked = { state = 1 },
                        onCameraClicked = { state = 0 },
                        screenWidth,
                        barHeight
                    )

                    when (state) {
                        0 -> CameraPage(screenHeight, barHeight)
                        1 -> ConfigurationPage()
                    }

                    /*
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.Absolute.Center
                    ) {
                        val titles = listOf("Camera", "Configuration")

                        titles.forEachIndexed { index, item ->
                            SegmentedButtonFun(
                                onClick = { state = index },
                                state = state,
                                index = index,
                                item = item,
                            )
                        }
                    }
                    */
                }
            }
        }
    }
}

@Composable
fun TopBar(
    onConfigurationClicked: () -> Unit,
    onCameraClicked: () -> Unit,
    screenWidth: Dp,
    barHeight: Dp,
) {

    Row(
        modifier = Modifier
            .height(barHeight)
            .width(screenWidth)
            .background(Color(0xFFD9D9D9).copy(alpha = 0.6f))
            .padding(10.dp),
        horizontalArrangement = Arrangement.Absolute.SpaceEvenly,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Row(modifier = Modifier.weight(1f)) {
            Image(
                painter = painterResource(id = R.drawable.configuration_button),
                contentDescription = "configuration",
                Modifier
                    .clickable(onClick = onConfigurationClicked)
            )
        }
        Row(modifier = Modifier.weight(1f),
            horizontalArrangement = Arrangement.Center) {
            Text(
                text = "AIDA Remote Control Beta",
                fontSize = 12.sp
            )
        }
        Row(modifier = Modifier.weight(1f),
            horizontalArrangement = Arrangement.End,
            verticalAlignment = Alignment.CenterVertically) {
            Spacer(Modifier.weight(5f))
            Image(
                painter = painterResource(id = R.drawable.camera_button),
                contentDescription = "camera",
                Modifier
                    .clickable(onClick = onCameraClicked)
            )
            Spacer(Modifier.weight(1f))
            Image(
                painter = painterResource(id = R.drawable.volume_button),
                contentDescription = "volume",
            )
            Spacer(Modifier.weight(1f))
            Text(
                text = "Battery: 30%",
                fontSize = 12.sp
            )
            Spacer(Modifier.weight(1f))
            Text(
                text = "Lag: 59ms",
                fontSize = 12.sp
            )
        }
    }
}

@Composable
fun SegmentedButtonFun(
    onClick: () -> Unit,
    state: Int,
    index: Int,
    item: String
) {
    val cornerRadius = 16.dp
    val configuration = LocalConfiguration.current

    val screenWidth = configuration.screenWidthDp.dp

    OutlinedButton(
        onClick = onClick,
        modifier = Modifier
            .width(screenWidth / 6)
            .offset(
                x = if (index == 0) 0.dp else (-1 * index).dp,
                y = 0.dp
            )
            .zIndex(if (state == index) 1f else 0f)
            .wrapContentHeight(),
        shape = RoundedCornerShape(
            topStart = if (index == 0) cornerRadius else 0.dp,
            topEnd = if (index == 0) 0.dp else cornerRadius,
            bottomStart = if (index == 0) cornerRadius else 0.dp,
            bottomEnd = if (index == 0) 0.dp else cornerRadius
        ),
        border = BorderStroke(
            1.dp, if (state == index) {
                MaterialTheme.colorScheme.primary
            } else {
                MaterialTheme.colorScheme.primary.copy(alpha = 0.75f)
            }
        ),
        colors = ButtonDefaults.outlinedButtonColors(
            containerColor = if (state == index)
                MaterialTheme.colorScheme.primary.copy(alpha = 0.6f) else
                MaterialTheme.colorScheme.surface.copy(alpha = 0.6f),
            contentColor = if (state == index)
                Color.White else
                MaterialTheme.colorScheme.primary
        )
    ) {
        Text(item)
    }
}
