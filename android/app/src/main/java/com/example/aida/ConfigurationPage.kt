package com.example.aida

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.text.KeyboardOptions
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
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp


/**
 * The configuration page contains information on how to connect to AIDA
 * It's still in early development and might therefore change, currently
 * there is no logic and only an UI component
 *
 * @param barHeight used to ensure that the content is padded correctly
 * @author Elias
 */
@Composable
fun ConfigurationPage(
    barHeight: Dp,
    ipAddress: String,
    onIpAddressChange: (String) -> Unit,
    port: Int,
    onPortChange: (Int) -> Unit,
    onButtonPress: () -> Unit
) {
    Row(
        modifier = Modifier
            .padding(top = barHeight)
            .fillMaxSize(),
    ) {
        val paddingTop = 20.dp
        val paddingSides = 30.dp
        val rowSpacing = 10.dp

        // First column for SSH Connection Data
        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding(top = paddingTop, start = paddingSides, end = paddingSides),
            verticalArrangement = Arrangement.spacedBy(rowSpacing),
            horizontalAlignment = Alignment.Start
        ) {
            var ipInput   by remember { mutableStateOf(ipAddress) }
            var portInput by remember { mutableStateOf(port.toString()) }

            Text(text = "SSH Connection Data", modifier = Modifier.align(Alignment.Start))
            Row(
                horizontalArrangement = Arrangement.spacedBy(rowSpacing)
            ) {
                TextField(
                    value = ipInput,
                    onValueChange = { newValue ->
                        // Check that the address is correctly formatted
                        if (newValue.matches(Regex("^((25[0-5]|(2[0-4]|1\\d|[1-9]|)\\d)\\.?\\b){4}\$"))) {
                            // Check each segment is within 0-255
                            ipInput = newValue
                        }
                    },
                    label = { Text("IP-address") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = Modifier.weight(1f)
                )
                TextField(
                    value = portInput,
                    onValueChange = { newValue ->
                        // Ensure the input is 0 to 4 characters
                        if (newValue.matches(Regex("^[0-9]{1,4}\$"))) {
                            portInput = newValue
                        }
                    },
                    label = { Text("Port") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = Modifier.weight(1f)
                )
            }
            Row(
                horizontalArrangement = Arrangement.spacedBy(rowSpacing)
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
                onClick = {
                    onIpAddressChange(ipInput)
                    onPortChange(portInput.toInt())
                    onButtonPress()
                },
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

        // Second column for the SSH Terminal
        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding(top = paddingTop, start = paddingSides, end = paddingSides),
            verticalArrangement = Arrangement.spacedBy(rowSpacing),
            horizontalAlignment = Alignment.Start
        ) {
            Text(text = "SSH Terminal", modifier = Modifier.align(Alignment.Start))
        }
    }
}