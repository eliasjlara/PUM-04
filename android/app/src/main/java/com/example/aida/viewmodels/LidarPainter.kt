package com.example.aida.viewmodels

import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import kotlin.math.cos
import kotlin.math.sin

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
        val centerX = width / 2f
        val centerY = height / 2f


        val paint = Paint()
        paint.color = Color.BLACK
        paint.strokeWidth = 3f

        for ((index, distance) in lidarValues.withIndex()) {
            val angle = Math.toRadians(index.toDouble())
            val normalizedDistanceX = distance.toFloat() / (maxLidarDistance/centerX)
            val normalizedDistanceY = distance.toFloat() / (maxLidarDistance/centerY)

            val x = centerX + normalizedDistanceX * cos(angle).toFloat()
            val y = centerY + normalizedDistanceY * sin(angle).toFloat()

            if (x >= 0 && x < width && y >= 0 && y < height) {
                canvas.drawPoint(x, y, paint)
            }
        }
        return bitmap
    }
}