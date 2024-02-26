package com.example.aida

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.CardDefaults.shape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.alpha
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.layout.ContentScale
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp

@Composable
fun CameraPage() {
    // Content for Camera tab

    Box(
        modifier = Modifier
            .clip(shape)
            .fillMaxSize()
    ) {
        Text("Camera view", Modifier.align(Alignment.Center))

        Image(
            painter = painterResource(id = R.drawable.kebab_palatset),
            contentDescription = "Hello world",
            contentScale = ContentScale.Crop,
            modifier = Modifier.fillMaxSize()
        )
        Box(
            modifier = Modifier
                .padding(40.dp)
                .clip(shape)
                .border(BorderStroke(3.dp, Color.Black))
                .background(Color.DarkGray.copy(0.6F))
                .alpha(0.6F)
                .size(100.dp)
                .align(Alignment.BottomStart),
            contentAlignment = Alignment.Center
        ) {
            Text(
                "Widget 1",
                color = Color.White,
                textAlign = TextAlign.Center
            )
        }

        Box(
            modifier = Modifier
                .padding(40.dp)
                .clip(shape)
                .border(BorderStroke(3.dp, Color.Black))
                .background(Color.DarkGray.copy(0.6F))
                .alpha(0.6F)
                .size(100.dp)
                .align(Alignment.BottomEnd),
            contentAlignment = Alignment.Center
        ) {
            Text(
                "Widget 2",
                color = Color.White
            )
        }
    }
}