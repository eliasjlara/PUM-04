package com.example.aida

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.offset
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
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.unit.dp
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
                    var state by remember { mutableStateOf(0) }

                    // Segmented Buttons Row, handles the change of tabs
                    when (state) {
                        0 -> CameraPage()
                        1 -> ConfigurationPage()
                    }
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
                }
            }
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
            .width(screenWidth/6)
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
