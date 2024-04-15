package com.example.aida

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.MenuBook
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.DrawerValue
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.ModalDrawerSheet
import androidx.compose.material3.ModalNavigationDrawer
import androidx.compose.material3.NavigationDrawerItem
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.rememberDrawerState
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.scale
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.aida.ui.theme.AIDATheme
import kotlinx.coroutines.launch


class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            AIDATheme {

                var state by remember { mutableStateOf(0) }

                val drawerState = rememberDrawerState(initialValue = DrawerValue.Closed)
                val scope = rememberCoroutineScope()

                var topBarTitle by remember { mutableStateOf("AIDA Remote Control Beta") }

                ModalNavigationDrawer(
                    drawerState = drawerState,
                    drawerContent = {
                        ModalDrawerSheet {
                            Text("Menu", modifier = Modifier.padding(16.dp))
                            HorizontalDivider()
                            NavigationDrawerItem(
                                icon = { Icon(Icons.Filled.Home, contentDescription = "Home") },
                                label = { Text(text = "Home") },
                                selected = false,
                                onClick = {
                                    state = 0
                                    scope.launch {
                                        drawerState.close()
                                    }
                                    topBarTitle = "AIDA Remote Control Beta"
                                }
                            )
                            NavigationDrawerItem(
                                icon = {
                                    Icon(
                                        Icons.Filled.Settings,
                                        contentDescription = "Settings"
                                    )
                                },
                                label = { Text(text = "Connection settings") },
                                selected = false,
                                onClick = {
                                    state = 1
                                    scope.launch {
                                        drawerState.close()
                                    }
                                    topBarTitle = "Connection settings"
                                }
                            )
                            NavigationDrawerItem(
                                icon = {
                                    Icon(
                                        Icons.AutoMirrored.Filled.MenuBook,
                                        contentDescription = "User Guide"
                                    )
                                },
                                label = { Text(text = "User guide") },
                                selected = false,
                                onClick = { /*TODO*/ }
                            )
                        }
                    },
                ) {
                    Surface(
                        color = MaterialTheme.colorScheme.background
                    ) {
                        val configuration = LocalConfiguration.current

                        val screenHeight = configuration.screenHeightDp.dp
                        val screenWidth = configuration.screenWidthDp.dp

                        val barHeight = if (screenHeight / 8 < 50.dp) screenHeight / 8 else 50.dp

                        TopBar(
                            onConfigurationClicked = {
                                scope.launch {
                                    drawerState.apply {
                                        if (isClosed) open() else close()
                                    }
                                }
                            },
                            onCameraClicked = { state = 0 },
                            screenWidth,
                            barHeight,
                            topBarTitle
                        )

                        when (state) {
                            0 -> CameraPage(screenHeight, barHeight, screenWidth)
                            1 -> ConfigurationPage(barHeight)
                        }
                    }
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
    topBarTitle: String,
) {
    Row(
        modifier = Modifier
            .height(barHeight)
            .width(screenWidth)
            .background(Color(0xFFD9D9D9).copy(alpha = 0.6f))
            .padding(start = 15.dp, end = 15.dp),
        horizontalArrangement = Arrangement.Absolute.SpaceEvenly,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Row(
            modifier = Modifier
                .weight(1f)
        ) {
            Image(
                painter = painterResource(id = R.drawable.configuration_button),
                contentDescription = "configuration",
                Modifier
                    .clickable(onClick = onConfigurationClicked)
                    .scale(1.2f)
            )
        }
        Row(
            modifier = Modifier.weight(1f),
            horizontalArrangement = Arrangement.Center
        ) {
            Text(
                text = topBarTitle,
                fontSize = 16.sp
            )
        }
        Row(
            modifier = Modifier.weight(1f),
            horizontalArrangement = Arrangement.End,
            verticalAlignment = Alignment.CenterVertically
        ) {
            Spacer(Modifier.weight(5f))
            Image(
                painter = painterResource(id = R.drawable.camera_button),
                contentDescription = "camera",
                Modifier
                    .clickable(onClick = onCameraClicked)
                    .scale(1.2f)
            )
            Spacer(Modifier.weight(1f))
            Image(
                painter = painterResource(id = R.drawable.volume_button),
                contentDescription = "volume",
                Modifier
                    .clickable(onClick = { /* TODO */ })
                    .scale(1.2f)
            )
            Spacer(Modifier.weight(1f))
            Text(
                text = "Battery: " + (1..100).random() + "%",
                fontSize = 16.sp
            )
            Spacer(Modifier.weight(1f))

            Text(
                text = "Lag: " + (30..100).random() + "ms",
                fontSize = 16.sp
            )
        }
    }
}