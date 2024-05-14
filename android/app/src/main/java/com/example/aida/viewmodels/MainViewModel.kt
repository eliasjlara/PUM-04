package com.example.aida.viewmodels

import android.graphics.BitmapFactory
import androidx.compose.ui.graphics.ImageBitmap
import androidx.compose.ui.graphics.asImageBitmap
import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.aida.enums.ConnectionStages
import com.example.aida.socketcommunication.LidarClient
import com.example.aida.socketcommunication.STTClient
import com.example.aida.socketcommunication.VideoClient
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.filter
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

/**
 * Contains all business logic for the application, i.e, it connects to
 * AIDA, fetches and sends information. It also contains the cached net-
 * work information like ip and port.
 *
 * Quick info regarding network requests:
 *
 *      viewModelScope.launch(Dispatchers.IO) is used to start a new
 *      thread, coroutine, since network requests can't be started on
 *      the main thread.
 *
 *      withContext(Dispatchers.Main) is used to go back to the main
 *      thread, for example when we want to update the STT string*
 *
 * For the future: This viewModel might be split up since it's quite
 * large
 *
 * @param dataStore contains cached information, is set in the viewModel-
 * factory
 *
 * @author Elias
 */
class MainViewModel(private val dataStore: DataStore<Preferences>) : ViewModel() {

    // Cached information
    private val IP_ADDRESS = stringPreferencesKey("ip_address")
    private val PORT = intPreferencesKey("port")

    // Use StateFlow for asynchronous updates and collecting them in Compose
    val ipAddress: StateFlow<String> = dataStore.data
        .map { preferences -> preferences[IP_ADDRESS] ?: "1.1.1.1" }
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(), "1.1.1.1")

    val port: StateFlow<Int> = dataStore.data
        .map { preferences -> preferences[PORT] ?: 1 }
        .stateIn(viewModelScope, SharingStarted.WhileSubscribed(), 1)

    // Wait til cache has loaded, then try to connect to AIDA
    init {
        viewModelScope.launch {
            ipAddress.filter { it != "1.1.1.1" }
                .collectLatest { ip ->
                    port.filter { it != 1 }
                        .collectLatest { prt ->
                            connectToAIDA(ip, prt)
                        }
                }
        }
    }

    fun updateIpAddress(newIp: String) {
        viewModelScope.launch {
            dataStore.edit { settings -> settings[IP_ADDRESS] = newIp }
        }
    }

    fun updatePort(newPort: Int) {
        viewModelScope.launch {
            dataStore.edit { settings -> settings[PORT] = newPort }
        }
    }

    // Init Clients
    private lateinit var sttClient: STTClient
    private lateinit var videoClient: VideoClient
    //private lateinit var lidarClient: LidarClient

    // Speech to text variables
    private val _voiceCommand = MutableLiveData<String>()
    val voiceCommand: LiveData<String> = _voiceCommand
    private val _isRecording = MutableLiveData<Boolean>()
    val isRecording: LiveData<Boolean> = _isRecording

    fun startVoiceRecording() {
        _isRecording.value = true

        viewModelScope.launch(Dispatchers.IO) {
            try {
                sttClient.sendStartMic()
                delay(10000) // Wait for STT to be ready
                sttClient.sendRequestSTTData()
                val result = sttClient.fetch()

                withContext(Dispatchers.Main) {
                    _voiceCommand.value = "\"" + String(result) + "\""
                    delay(5000)
                    _voiceCommand.value = "Carrying out the action in sequence"
                    delay(5000)
                    _isRecording.value = false
                    delay(1000)
                    _voiceCommand.value = "I am listening ..."
                }
            } catch (e: Exception) {
                println("MIC ERROR: $e")

                withContext(Dispatchers.Main) {
                    delay(5000)
                    _voiceCommand.value = "Something went wrong, try again"
                    delay(5000)
                    _isRecording.value = false
                }

            }
        }
    }

    // Connections to AIDA
    private val _cameraFeedConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    private val _lidarConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    private val _sttConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    private val _joystickConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    private val _imageBitmap = MutableStateFlow<ImageBitmap?>(null)

    val cameraFeedConnectionStage: StateFlow<ConnectionStages> =
        _cameraFeedConnectionStage.asStateFlow()
    val lidarConnectionStage: StateFlow<ConnectionStages> = _lidarConnectionStage.asStateFlow()
    val sttConnectionStage: StateFlow<ConnectionStages> = _sttConnectionStage.asStateFlow()
    val joystickConnectionStage: StateFlow<ConnectionStages> =
        _joystickConnectionStage.asStateFlow()
    val imageBitmap: StateFlow<ImageBitmap?> = _imageBitmap.asStateFlow()


    fun connectToAIDA(ip: String = ipAddress.value, prt: Int = port.value) {
        viewModelScope.launch(Dispatchers.IO) {
            _cameraFeedConnectionStage.value = ConnectionStages.CONNECTING
            _lidarConnectionStage.value = ConnectionStages.CONNECTING
            _sttConnectionStage.value = ConnectionStages.CONNECTING

            connectToSTT(ip, prt)
            connectToVideo(ip, prt)

            // TODO Implement lidar fetch
            connectToLidar(ip, prt)

            // TODO Implement send joystick
            connectToJoystick()

        }
    }

    private fun connectToVideo(ip: String, prt: Int) {
        viewModelScope.launch(Dispatchers.IO) {
            try {
                videoClient = VideoClient(
                    ip = ip,
                    port = prt,
                    timeToTimeout = 10000
                )
                videoClient.sendStartCamera()
                videoClient.sendGetVideo()

                _cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED

                while (true) {
                    val byteArray = videoClient.fetch()
                    _imageBitmap.value =
                        BitmapFactory.decodeByteArray(byteArray, 0, byteArray.size)
                            ?.asImageBitmap()
                }
            } catch (e: Exception) {
                println("Can't Connect to Video: $e")
                _cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
        }
    }

    private fun connectToSTT(ip: String, prt: Int) {
        viewModelScope.launch(Dispatchers.IO) {
            try {
                sttClient = STTClient(
                    ip = ip,
                    port = prt,
                    timeToTimeout = 5000
                )
                sttClient.sendStartSTT()

                _sttConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED
            } catch (e: Exception) {
                println("Can't connect to STT: $e")
                _cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
        }
    }

    private fun connectToLidar(ip : String, prt : Int) {
        viewModelScope.launch(Dispatchers.IO) {
            try {
                /*lidarClient = LidarClient(
                    ip = ip,
                    port = prt,
                    timeToTimeout = 5000
                )
                lidarClient.sendStartLidar()
                lidarClient.sentRequestLidarData()*/
                _lidarConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED
                //while (true) {
                    // TODO - Implement fetch
                    //val byteArray = lidarClient.fetch()
                    //_imageBitmap.value =
                    //    BitmapFactory.decodeByteArray(byteArray, 0, byteArray.size)
                    //        ?.asImageBitmap()
                //}
            } catch (e: Exception) {
                println("Can't connect to Lidar: $e")
                _lidarConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
        }
    }

    private fun connectToJoystick() {
        viewModelScope.launch(Dispatchers.IO) {
            try {
                _joystickConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED
            } catch (e: Exception) {
                println("Can't connect to Lidar: $e")
                _joystickConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
        }
    }

    fun closeConnections() {
        try {
            sttClient.stop()
            videoClient.stop()
            //lidarClient.stop()
        } catch (e: Exception) {
            println("Error when closing: $e")
        }
    }
}
