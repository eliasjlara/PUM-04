package com.example.aida

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.aida.socketcommunication.STTClient
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

class SpeechViewModel : ViewModel() {
    private val _voiceCommand = MutableLiveData<String>()
    val voiceCommand: LiveData<String> = _voiceCommand

    private lateinit var sttClient: STTClient

    fun initializeSTTClient(ip: String, port: Int, timeToTimeout : Int) {
        sttClient = STTClient(ip, port, timeToTimeout)
    }

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

    override fun onCleared() {
        super.onCleared()
        sttClient.stop() // Clean up resources
    }
}