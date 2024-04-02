package com.example.aida

import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier

@Composable
fun ConfigurationPage() {
    // Content for Configuration tab

    Box(modifier = Modifier.fillMaxSize()) {
        Text("Configuration Page Content", Modifier.align(Alignment.Center))
    }
}