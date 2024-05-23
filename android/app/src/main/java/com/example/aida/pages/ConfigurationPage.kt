package com.example.aida.pages

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
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TextField
import androidx.compose.material3.VerticalDivider
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import com.example.aida.viewmodels.MainViewModel

/**
 * The configuration page contains information on how to connect to AIDA
 * It's still in early development and might therefore change, currently
 * there is no logic and only an UI component
 *
 * @param barHeight used to ensure that the content is padded correctly
 * @param viewModel contains connection information, used to update IP
 * address, update port and connect to AIDA with new information
 * @param onButtonPress updates the state and the topBar title
 *
 * @author Elias
 */
@Composable
fun ConfigurationPage(
    barHeight: Dp,
    viewModel: MainViewModel,
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
            verticalArrangement = Arrangement.spacedBy(rowSpacing)
        ) {
            var ipInput by remember { mutableStateOf(viewModel.ipAddress.value) }
            var portInput by remember { mutableStateOf(viewModel.port.value.toString()) }
            val standardInputModifier = Modifier.weight(1f)

            Text(text = "SSH Connection Data", modifier = Modifier.align(Alignment.Start))

            var isIpError by rememberSaveable { mutableStateOf(false) }
            var isPortError by rememberSaveable { mutableStateOf(false) }

            // IP & Port input
            Row(
                horizontalArrangement = Arrangement.spacedBy(rowSpacing)
            ) {
                // IP input
                TextField(
                    value = ipInput,
                    onValueChange = { newValue ->
                        // Check that the address is correctly formatted
                        ipInput = newValue
                        isIpError =
                            !ipInput.matches(Regex("^((25[0-5]|(2[0-4]|1\\d|[1-9]|)\\d)\\.?\\b){4}\$"))
                    },
                    label = { Text("IP-address") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = standardInputModifier,
                    isError = isIpError,
                    supportingText = {
                        if (isIpError) {
                            Text(
                                modifier = Modifier.fillMaxWidth(),
                                text = "IP-address is not correctly formatted, should be of standard \"1.1.1.1\"",
                                color = MaterialTheme.colorScheme.error
                            )
                        }
                    },
                )

                // Port input
                TextField(
                    value = portInput,
                    onValueChange = { newValue ->
                        // Ensure the input is 0 to 4 characters
                        portInput = newValue
                        isPortError = !newValue.matches(Regex("^[0-9]{1,4}\$"))
                    },
                    label = { Text("Port") },
                    keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                    singleLine = true,
                    modifier = standardInputModifier,
                    isError = isPortError,
                    supportingText = {
                        if (isPortError) {
                            Text(
                                modifier = Modifier.fillMaxWidth(),
                                text = "Port is not correctly formatted, should be of standard 1 to 9999",
                                color = MaterialTheme.colorScheme.error
                            )
                        }
                    },
                )
            }

            // Username & Password input, currently unused
            Row(
                horizontalArrangement = Arrangement.spacedBy(rowSpacing)
            ) {
                var username by remember { mutableStateOf("") }
                var password by remember { mutableStateOf("") }

                // Username input, currently unused
                TextField(
                    value = username,
                    onValueChange = { username = it },
                    label = { Text("Username") },
                    modifier = standardInputModifier
                )

                // Password input, currently unused
                TextField(
                    value = password,
                    onValueChange = { password = it },
                    label = { Text("Password") },
                    modifier = standardInputModifier
                )
            }

            // Button to confirm IP address and port
            Button(
                onClick = {
                    if (!isIpError || !isPortError) {
                        viewModel.updateIpAddress(ipInput)
                        viewModel.updatePort(portInput.toInt())
                        viewModel.connectToAIDA()
                        onButtonPress()
                    }
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

        // Second column for the SSH Terminal, currently unused
        Column(
            modifier = Modifier
                .fillMaxWidth(0.5f)
                .padding(top = paddingTop, start = paddingSides, end = paddingSides),
            verticalArrangement = Arrangement.spacedBy(rowSpacing)
        ) {
            Text(
                text = "SSH Terminal"
            )
        }
    }
}
