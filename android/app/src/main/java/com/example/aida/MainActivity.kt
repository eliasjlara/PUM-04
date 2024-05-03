package com.example.aida

import android.content.Context
import android.graphics.BitmapFactory
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.offset
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
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.scale
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.preferencesDataStore
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.aida.socketcommunication.VideoClient
import com.example.aida.ui.theme.AIDATheme
import com.example.aida.ui.theme.TopBarColor
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

/**
 * MainActivity for application. Displays a top bar that contains a menu
 * and different widgets as well as the main application. The main switches
 * between a camera/lidar feed and configuration page
 *
 * TODO: Fetch real data for camera feed
 * TODO: Fetch real data for lidar feed
 * TODO: Use classes for camera and configuration page?
 * TODO: Decide on what widgets in top bar should do
 *
 * @author Elias
 */
class MainActivity : ComponentActivity() {
    private val Context.dataStore: DataStore<Preferences> by preferencesDataStore(name = "settings")

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContent {

            /*
            val context = LocalContext.current

            val IP_ADDRESS = stringPreferencesKey("ip_address")
            val PORT = intPreferencesKey("port")

            val cachedIpAddress: Flow<String> = context.dataStore.data
                .map { preferences ->
                    // No type safety.
                    preferences[IP_ADDRESS] ?: "1.1.1.1"
                }
            val cachedPort: Flow<Int> = context.dataStore.data
                .map { preferences ->
                    // No type safety.
                    preferences[PORT] ?: 1
                }

            suspend fun updateIpAddress(ipAddress: String) {
                context.dataStore.edit { settings ->
                    settings[IP_ADDRESS] = ipAddress
                }
            }

            suspend fun updatePort(port: Int) {
                context.dataStore.edit { settings ->
                    settings[PORT] = port
                }
            }

            val ipAddress by cachedIpAddress.collectAsState(initial = "1.1.1.1")
            val port by cachedPort.collectAsState(initial = 1)
             */

            val context = LocalContext.current
            val dataStore = context.dataStore
            val viewModel: MainViewModel = viewModel(factory = MainViewModelFactory(dataStore))

            val ipAddress by viewModel.ipAddress.collectAsState()
            val port by viewModel.port.collectAsState()


            var cameraFeedConnectionStage by remember { mutableStateOf(ConnectionStages.CONNECTING) }
            var lidarConnectionStage by remember { mutableStateOf(ConnectionStages.CONNECTING) }
            var sttConnectionStage by remember { mutableStateOf(ConnectionStages.CONNECTING) }
            var imageBitmap by remember { mutableStateOf<ImageBitmap?>(null) }
            val speechViewModel = SpeechViewModel()

            suspend fun connectToAIDA() {
                cameraFeedConnectionStage = ConnectionStages.CONNECTING
                lidarConnectionStage = ConnectionStages.CONNECTING
                sttConnectionStage = ConnectionStages.CONNECTING

                withContext(Dispatchers.IO) {
                    try {
                        val videoClient = VideoClient(
                            ip = ipAddress,
                            port = port,
                            timeToTimeout = 10000
                        )
                        videoClient.sendStartCamera()
                        videoClient.sendGetVideo()

                        cameraFeedConnectionStage = ConnectionStages.CONNECTION_SUCCEEDED

                        while (true) {
                            val byteArray = videoClient.fetch()
                            imageBitmap =
                                BitmapFactory.decodeByteArray(byteArray, 0, byteArray.size)
                                    ?.asImageBitmap()
                        }
                    } catch (e: Exception) {
                        println("Network Custom Exception: $e")
                        cameraFeedConnectionStage = ConnectionStages.CONNECTION_FAILED
                    }
                    try {
                        speechViewModel.initializeSTTClient(
                            ip = ipAddress,
                            port = port,
                            timeToTimeout = 5000
                        )
                        sttConnectionStage = ConnectionStages.CONNECTION_SUCCEEDED
                    } catch (e: Exception) {
                        // TODO Remove lidar line below, only used for testing
                        lidarConnectionStage = ConnectionStages.CONNECTION_FAILED
                        sttConnectionStage = ConnectionStages.CONNECTION_FAILED
                        print("Speech Exception: $e")
                    }
                    // TODO Implement lidar fetch
                }
            }

            LaunchedEffect(Unit) {
                connectToAIDA()
            }

            AIDATheme {
                var topBarTitle by remember { mutableStateOf("AIDA Remote Control Beta") }
                val drawerState = rememberDrawerState(initialValue = DrawerValue.Closed)
                var state by remember { mutableIntStateOf(0) }
                val scope = rememberCoroutineScope()

                // Setup menu
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
                        // Dynamically set bar height depending on phone/tablet
                        val barHeight = if (screenHeight / 8 < 50.dp) screenHeight / 6 else 50.dp

                        TopBar(
                            onMenuClicked = {
                                scope.launch {
                                    drawerState.apply {
                                        if (isClosed) open() else close()
                                    }
                                }
                            },
                            onCameraClicked = { state = 0 },
                            screenWidth,
                            barHeight,
                            topBarTitle = topBarTitle
                        )

                        val configurationCoroutineScope = rememberCoroutineScope()

                        when (state) {
                            0 -> CameraPage(
                                screenHeight,
                                barHeight,
                                screenWidth,
                                imageBitmap,
                                cameraFeedConnectionStage,
                                speechViewModel,
                                sttConnectionStage,
                                lidarConnectionStage,
                                ipAddress,
                                port
                            )

                            1 -> ConfigurationPage(
                                barHeight,
                                ipAddress = ipAddress,
                                onIpAddressChange = { newIp ->
                                    configurationCoroutineScope.launch {
                                        viewModel.updateIpAddress(newIp)
                                    }
                                },
                                port = port,
                                onPortChange = { newPort ->
                                    configurationCoroutineScope.launch {
                                        viewModel.updatePort(newPort)
                                    }
                                },
                                onButtonPress = {
                                    state = 0

                                    configurationCoroutineScope.launch {
                                        connectToAIDA()
                                    }
                                    topBarTitle = "AIDA Remote Control Beta"
                                }
                            )
                        }
                    }
                }
            }
        }
    }
}

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
    screenWidth: Dp,
    barHeight: Dp,
    topBarTitle: String,
) {
    val barPadding = 15.dp

    Row(
        modifier = Modifier
            .height(barHeight)
            .width(screenWidth)
            .background(TopBarColor.copy(alpha = 0.6f))
            .padding(start = barPadding, end = barPadding),
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
                    .clickable(onClick = onMenuClicked)
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
            var cameraPress by remember { mutableStateOf("on") }
            var speakerPress by remember { mutableStateOf("on") }

            Spacer(Modifier.weight(5f))

            Column(
                horizontalAlignment = Alignment.CenterHorizontally, // Aligns the contents of the column to the center horizontally
                modifier = Modifier
                    .clickable(onClick = {
                        cameraPress = if (cameraPress == "on") "off" else "on"
                        onCameraClicked()
                    }
                    )
                    .padding(8.dp) // Adds padding around the column
            ) {
                Image(
                    painter = painterResource(id = R.drawable.camera_button),
                    contentDescription = "camera",
                    Modifier
                        .scale(1.2f)
                )
                Text(
                    text = cameraPress,
                    Modifier.offset(y = (-2).dp)
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
                    .padding(8.dp)
            ) {
                Image(
                    painter = painterResource(id = R.drawable.volume_button),
                    contentDescription = "camera",
                    Modifier
                        .scale(1.2f)
                )
                Text(
                    text = speakerPress,
                    Modifier.offset(y = (-2).dp)
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