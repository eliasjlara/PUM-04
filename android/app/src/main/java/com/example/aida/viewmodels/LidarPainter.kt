package com.example.aida.viewmodels

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import kotlin.math.cos
import kotlin.math.sin

/**
 * This class is used to create a LidarPainter object to convert Lidar data to Bitmap
 */
class LidarPainter(val width: Int, val height: Int, val maxLidarDistance: Int = 5000) {

    /**
     * Create a bitmap from the lidar values
     * @param lidarValues the lidar values to create the bitmap from
     * @return the bitmap created from the lidar values
     */
    fun createLiDARBitmap(lidarValues: IntArray): Bitmap {
        val bitmap = Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565)
        val canvas = Canvas(bitmap)
        canvas.drawColor(Color.WHITE)
        val paint = Paint()
        paint.color = Color.BLACK
        paint.strokeWidth = 3f

        for ((index, distance) in lidarValues.withIndex()) {
            val (x, y) = calculatePixelValue(distance, index.toDouble())
            // If pixel value is inside the bitmap, draw the point
            if (x >= 0 && x < width && y >= 0 && y < height) {
                canvas.drawPoint(x, y, paint)
            }
        }
        return bitmap
    }

    /**
     *  Calculate the pixel value for a given distance and angle
     *  @param distance the distance to calculate the pixel value for
     *  @param angleInDegrees the angle in degrees to calculate the pixel value for
     */
    fun calculatePixelValue(distance: Int, angleInDegrees: Double): Pair<Float, Float>{
        val angleInRadians = Math.toRadians(angleInDegrees)
        val normalizedDistanceX = distance.toFloat() / (maxLidarDistance/width)
        val normalizedDistanceY = distance.toFloat() / (maxLidarDistance/height)
        val x = width / 2.0f + normalizedDistanceX * cos(angleInRadians).toFloat()
        val y = height / 2.0f + normalizedDistanceY * sin(angleInRadians).toFloat()
        return Pair(x, y)
    }
}