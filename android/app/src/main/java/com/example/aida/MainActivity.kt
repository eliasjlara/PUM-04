package com.example.aida

import android.content.Context
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.layout.padding
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
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalConfiguration
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.preferencesDataStore
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.aida.pages.CameraPage
import com.example.aida.pages.ConfigurationPage
import com.example.aida.ui.composables.TopBar
import com.example.aida.ui.theme.AIDATheme
import com.example.aida.viewmodels.MainViewModel
import com.example.aida.viewmodels.MainViewModelFactory
import kotlinx.coroutines.launch

/**
 * MainActivity for application. Displays a top bar that contains a menu
 * and different widgets as well as the main application. The main switches
 * between a camera/lidar feed and configuration page
 *
 * TODO: Fetch real data for lidar feed
 * TODO: Decide on what widgets in top bar should do
 *
 * @author Elias
 */

// Set cache for connection settings
private val Context.dataStore: DataStore<Preferences> by preferencesDataStore(
    name = "settings"
)

class MainActivity : ComponentActivity() {

    // This viewModel contains all our business logic, i.e., socket connections
    // to AIDA and fetching from cache. We use late init so we can close connections later
    private lateinit var viewModel: MainViewModel

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        setContent {
            val context = LocalContext.current
            val dataStore = context.dataStore
            viewModel = viewModel(factory = MainViewModelFactory(dataStore))

            // Display UI
            AppContent(viewModel)
        }
    }

    // Not guaranteed to be called according to android docs
    override fun onDestroy() {
        viewModel.closeConnections()
        super.onDestroy()
    }

    // Contains all our UI
    @Composable
    fun AppContent(viewModel: MainViewModel) {
        AIDATheme {
            var topBarTitle by remember { mutableStateOf("AIDA Remote Control Beta") }
            val drawerState = rememberDrawerState(initialValue = DrawerValue.Closed)
            var state by remember { mutableIntStateOf(0) }
            val scope = rememberCoroutineScope()

            // Setup menu drawer
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
                        onCameraClicked = { viewModel.toggleCameraFeed() },
                        barHeight = barHeight,
                        topBarTitle = topBarTitle
                    )

                    // Main logic when switching between pages
                    when (state) {
                        0 -> CameraPage(
                            screenHeight = screenHeight,
                            barHeight = barHeight,
                            screenWidth =screenWidth,
                            viewModel = viewModel
                        )

                        1 -> ConfigurationPage(
                            barHeight = barHeight,
                            viewModel = viewModel,
                            onButtonPress = {
                                state = 0
                                topBarTitle = "AIDA Remote Control Beta"
                            }
                        )
                    }
                }
            }
        }
    }
}
