package com.example.aida

import androidx.datastore.core.DataStore
import androidx.datastore.preferences.core.Preferences
import androidx.datastore.preferences.core.edit
import androidx.datastore.preferences.core.intPreferencesKey
import androidx.datastore.preferences.core.stringPreferencesKey
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

/*
class com.example.aida.MainViewModel(application: Application) : AndroidViewModel(application) {
private val dataStore: DataStore<Preferences> by preferencesDataStore(name = "settings")
private val IP_ADDRESS = stringPreferencesKey("ip_address")
private val PORT = intPreferencesKey("port")

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

// Add methods for network communication and other logic here
}
 */

class MainViewModel(private val dataStore: DataStore<Preferences>) : ViewModel() {
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
}
