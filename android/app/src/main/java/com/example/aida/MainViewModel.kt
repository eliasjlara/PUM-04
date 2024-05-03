package com.example.aida

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
import com.example.aida.socketcommunication.STTClient
import com.example.aida.socketcommunication.VideoClient
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

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

    // Speech to Text things
    private val _voiceCommand = MutableLiveData<String>()
    val voiceCommand: LiveData<String> = _voiceCommand

    private val sttClient = STTClient(
        ip = ipAddress.value,
        port = port.value,
        timeToTimeout = 5000
    )

    fun startVoiceRecording() {
        viewModelScope.launch(Dispatchers.IO) {
            try {
                sttClient.sendStartSTT()
                delay(15000) // Wait for STT to be ready
                sttClient.sendRequestSTTData()
                val result = sttClient.fetch().toString()
                withContext(Dispatchers.Main) {
                    _voiceCommand.value = result
                }
            } catch (e: Exception) {
                withContext(Dispatchers.Main) {
                    _voiceCommand.value = "Error: ${e.message}"
                }
            }
        }
    }

    // Connections to AIDA
    val cameraFeedConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    val lidarConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    val sttConnectionStage = MutableStateFlow(ConnectionStages.CONNECTING)
    val imageBitmap = MutableStateFlow<ImageBitmap?>(null)

    fun connectToAIDA() {
        viewModelScope.launch(Dispatchers.IO) {
            cameraFeedConnectionStage.value = ConnectionStages.CONNECTING
            lidarConnectionStage.value = ConnectionStages.CONNECTING
            sttConnectionStage.value = ConnectionStages.CONNECTING

            try {
                val videoClient = VideoClient(
                    ip = ipAddress.value,
                    port = port.value,
                    timeToTimeout = 10000
                )
                videoClient.sendStartCamera()
                videoClient.sendGetVideo()

                cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED

                while (true) {
                    val byteArray = videoClient.fetch()
                    imageBitmap.value =
                        BitmapFactory.decodeByteArray(byteArray, 0, byteArray.size)
                            ?.asImageBitmap()
                }
            } catch (e: Exception) {
                println("Can't Connect to Video: $e")
                cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
            try {
                val sttClient = STTClient(
                    ip = ipAddress.value,
                    port = port.value,
                    timeToTimeout = 5000
                )
                sttClient.sendStartSTT()
                sttClient.sendRequestSTTData()

                sttConnectionStage.value = ConnectionStages.CONNECTION_SUCCEEDED
            } catch (e: Exception) {
                println("Can't connect to STT: $e")
                cameraFeedConnectionStage.value = ConnectionStages.CONNECTION_FAILED
            }
            // TODO Implement lidar fetch
        }
    }
}
