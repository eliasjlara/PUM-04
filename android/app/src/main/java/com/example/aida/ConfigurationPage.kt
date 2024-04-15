package com.example.aida

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Button
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.material3.VerticalDivider
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp


@Composable
fun ConfigurationPage(barHeight: Dp) {
    // Content for Configuration tab
    Row(
        modifier = Modifier
            .padding(top = barHeight)
            .fillMaxSize(),
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding(top = 20.dp, start = 30.dp, end = 30.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
            horizontalAlignment = Alignment.Start
        ) {
            Text(text = "SSH Connection Data", modifier = Modifier.align(Alignment.Start))
            Row(
                horizontalArrangement = Arrangement.spacedBy(10.dp)
            ) {
                var ip by remember { mutableStateOf("") }
                var port by remember { mutableStateOf("") }

                TextField(
                    value = ip,
                    onValueChange = { ip = it },
                    label = { Text("IP-address") },
                    modifier = Modifier.weight(1f)
                )
                TextField(
                    value = port,
                    onValueChange = { port = it },
                    label = { Text("Port") },
                    modifier = Modifier.weight(1f)
                )
            }
            Row(
                horizontalArrangement = Arrangement.spacedBy(10.dp)
            ) {
                var username by remember { mutableStateOf("") }
                var password by remember { mutableStateOf("") }

                TextField(
                    value = username,
                    onValueChange = { username = it },
                    label = { Text("Username") },
                    modifier = Modifier.weight(1f)
                )
                TextField(
                    value = password,
                    onValueChange = { password = it },
                    label = { Text("Password") },
                    modifier = Modifier.weight(1f)
                )
            }
            Button(
                onClick = { /*TODO*/ },
                modifier = Modifier
                    .padding(20.dp)
                    .align(Alignment.CenterHorizontally)
                    .fillMaxWidth(0.4f)
            ) {
                Text("Connect")
            }
        }

        VerticalDivider(
            modifier = Modifier
                .padding(top = 30.dp, bottom = 30.dp)
                .fillMaxHeight()
                .width(1.dp),
            color = Color.Gray
        )

        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding(top = 10.dp, start = 20.dp, end = 20.dp),
            verticalArrangement = Arrangement.spacedBy(10.dp),
            horizontalAlignment = Alignment.Start
        ) {
            var test by remember { mutableStateOf("") }
            Text(text = "SSH Terminal", modifier = Modifier.align(Alignment.Start))
        }
    }
}