package com.example.aida.viewmodels

import androidx.compose.ui.graphics.ImageBitmap
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
import com.example.aida.socketcommunication.JoystickClient
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
    private lateinit var lidarClient: LidarClient
    private lateinit var joystickClient: JoystickClient

    // Speech to text variables
    private val _voiceCommand = MutableLiveData<String>()
    val voiceCommand: LiveData<String> = _voiceCommand
    private val _isRecording = MutableLiveData<Boolean>()
    val isRecording: LiveData<Boolean> = _isRecording

    /**
     * Starts the voice recording process and fetches the STT result
     * from the server.
     */
    fun startVoiceRecording() {
        _isRecording.value = true

        viewModelScope.launch(Dispatchers.IO) {
            try {
                sttClient.sendStartMic()
                delay(10000) // Wait for STT to be ready
                sttClient.sendRequestSTTData()
                val result = sttClient.receiveSTTData()

                withContext(Dispatchers.Main) {
                    //_voiceCommand.value = "\"" + String(result) + "\""
                    _voiceCommand.value = "\"" + result + "\""
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
    // Joystick variables to send at specified intervals
    private var _sendingJoystickData = MutableLiveData<Boolean>(false)
    val sendingJoystickData: LiveData<Boolean> = _sendingJoystickData

    /**
     * Sends joystick data to the server at specified intervals.
     * @param x the x value of the joystick
     * @param y the y value of the joystick
     */
    fun sendJoystickData(x: Float, y: Float) {
        _sendingJoystickData.value = true
        viewModelScope.launch(Dispatchers.IO) {
            try {
                joystickClient.sendJoystickData(x, y)
                delay(1000)
                withContext(Dispatchers.Main) {
                    _sendingJoystickData.value = false
                }

            } catch (e: Exception) {
                println("Joystick error: $e")
                withContext(Dispatchers.Main) {
                    _sendingJoystickData.value = false
                }
            }
        }
    }


    // Connections to AIDA
    private val _cameraFeedConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    private val _lidarConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    private val _sttConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    private val _joystickConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    // Image bitmap for camera feed
    private val _videoBitmap = MutableStateFlow<ImageBitmap?>(null)
    // Image bitmap for lidar
    private val _lidarImageBitmap = MutableStateFlow<ImageBitmap?>(null)

    val cameraFeedConnectionStage: StateFlow<ConnectionStages> =
        _cameraFeedConnectionStage.asStateFlow()
    val lidarConnectionStage: StateFlow<ConnectionStages> = _lidarConnectionStage.asStateFlow()
    val sttConnectionStage: StateFlow<ConnectionStages> = _sttConnectionStage.asStateFlow()
    val joystickConnectionStage: StateFlow<ConnectionStages> =
        _joystickConnectionStage.asStateFlow()
    // Image bitmap for camera feed
    val videoBitmap: StateFlow<ImageBitmap?> = _videoBitmap.asStateFlow()
    // Image bitmap for lidar
    val lidarImageBitmap: StateFlow<ImageBitmap?> = _lidarImageBitmap.asStateFlow()

    /**
    * Toggles the camera feed on and off
    * If the camera feed is on, it will close the connection to server and turn of
    * video feed
    * If the camera feed is off, it will connect to the server and start the video feed
     */
    fun toggleCameraFeed(){
            if (_cameraFeedConnectionStage.value == ConnectionStages.CONNECTION_SUCCEEDED){
                viewModelScope.launch (Dispatchers.IO){
                    try {
                        videoClient.sendStopCamera()
                        videoClient.stop()
                        _videoBitmap.value = null
                        _cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_CLOSED
                    }
                    catch (e: Exception){
                        _cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_FAILED
                        println("Can't close video: $e")
                    }

                }

            }
            else if (_cameraFeedConnectionStage.value == ConnectionStages.CONNECTION_CLOSED){
                viewModelScope.launch (Dispatchers.IO){
                    _cameraFeedConnectionStage.value = ConnectionStages.CONNECTING
                    connectToVideo(ipAddress.value, port.value)
                }
            }
    }
    /**
    * Connects to AIDA API with all clients
    * @param ip the ip address of the server
    * @param prt the port of the server
    * If no ip and port is provided, it will use the cached values
     */
    fun connectToAIDA(ip: String = ipAddress.value, prt: Int = port.value) {
        viewModelScope.launch(Dispatchers.IO) {
            _cameraFeedConnectionStage.value = ConnectionStages.CONNECTING
            _lidarConnectionStage.value = ConnectionStages.CONNECTING
            _sttConnectionStage.value = ConnectionStages.CONNECTING
            _joystickConnectionStage.value = ConnectionStages.CONNECTING

            connectToSTT(ip, prt)
            connectToVideo(ip, prt)
            connectToLidar(ip, prt)
            connectToJoystick(ip, prt)
        }
    }

    /**
     * Connects to the video client and starts receiving video data
     */
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
                _videoBitmap.value = videoClient.receiveVideoData()
                _cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED

                while (_cameraFeedConnectionStage.value == ConnectionStages.CONNECTION_SUCCEEDED) {
                    _videoBitmap.value = videoClient.receiveVideoData()
                }
            } catch (e: Exception) {
                println("Can't Connect to Video: $e")
                // If we close the video data - we fetch from a closed connection. Set the
                // connection stage to closed, else set to failed.
                if (_cameraFeedConnectionStage.value != ConnectionStages.CONNECTION_CLOSED)
                    _cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
        }
    }
    /**
    * Connects the STT client to the server
     */
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

    /**
     * Connects to the Lidar client and starts receiving Lidar data
     */
    private fun connectToLidar(ip : String, prt : Int) {
        viewModelScope.launch(Dispatchers.IO) {
            try {
                lidarClient = LidarClient(
                    ip = ip,
                    port = prt,
                    timeToTimeout = 10000
                )
                lidarClient.sendStartLidar()
                lidarClient.sentRequestLidarData()
                _lidarImageBitmap.value = lidarClient.receiveLidarData()
                _lidarConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED
                while (true) {
                    _lidarImageBitmap.value = lidarClient.receiveLidarData()
                }
            } catch (e: Exception) {
                println("Can't connect to Lidar: $e")
                _lidarConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
        }
    }

    /**
     * Connects to the Joystick client
     */
    private fun connectToJoystick(ip: String, prt: Int) {
        viewModelScope.launch(Dispatchers.IO) {
            try {
                joystickClient = JoystickClient(
                    ip = ip,
                    port = prt,
                    timeToTimeout = 5000
                )
                _joystickConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED
            } catch (e: Exception) {
                println("Can't connect to Lidar: $e")
                _joystickConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
        }
    }

    /**
     * Closes all connections to AIDA
     */
    fun closeConnections() {
        try {
            sttClient.stop()
            videoClient.stop()
            lidarClient.stop()
            joystickClient.stop()
        } catch (e: Exception) {
            println("Error when closing: $e")
        }
    }
}
