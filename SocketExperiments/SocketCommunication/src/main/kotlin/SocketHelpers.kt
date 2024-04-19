package org.example

import java.io.File
import javax.imageio.ImageIO
import javax.swing.ImageIcon
import javax.swing.JFrame
import javax.swing.JLabel




fun getByteArrayFromPath(path: String): ByteArray {
    return File(path).readBytes()
}

/**
 * This function gets the file from the path and reads the bytes from file
 * The file must be located in the resources folder of the project
 */
//fun getByteArrayFromPath(path: String): ByteArray {

//}

class ImagePainter(){
    private val frame = JFrame("Image Display")
    private val label = JLabel()

    fun setDefaultCloseOperation(){
        frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    }

    fun displayImageFromByteArray(imageData: ByteArray) {
        // Read the image from the byte array
        val image = ImageIO.read(imageData.inputStream())

        label.icon = ImageIcon(image)
        frame.contentPane.add(label)

        // Pack the frame and set it visible
        frame.pack()
        frame.isVisible = true
    }
}