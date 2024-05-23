package com.example.aida.viewmodels

import org.junit.Assert.assertEquals
import org.junit.Test

class LidarPainterTest {

    @Test
    fun calculatePixelValueCompleteY() {
        val lidarPainter = LidarPainter(100, 100, 5000)
        val (x, y) = lidarPainter.calculatePixelValue(100, 90.0)
        assertEquals(50.0f, x)
        assertEquals(52.0f, y)
    }

    @Test
    fun calculatePixelValueCompleteX() {
        val width = 100
        val height = 100
        val lidarPainter = LidarPainter(width, height, 5000)
        val (x, y) = lidarPainter.calculatePixelValue(100, 0.0)
        assertEquals(52.0f, x)
        assertEquals(50.0f, y)
    }

    @Test
    fun calculatePixelValue(){
        val lidarPainter = LidarPainter(1000, 1000, 5000)
        val (x, y) = lidarPainter.calculatePixelValue(100, 45.0)
        assertEquals(514.14215f, x)
        assertEquals(514.14215f, y)
    }

    @Test
    fun getWidth() {
        val lidarPainter = LidarPainter(100, 100, 5000)
        assertEquals(100, lidarPainter.width)
    }

    @Test
    fun getHeight() {
        val lidarPainter = LidarPainter(100, 100, 5000)
        assertEquals(100, lidarPainter.height)
    }

    @Test
    fun getMaxLidarDistance() {
        val lidarPainter = LidarPainter(100, 100, 5000)
        assertEquals(5000, lidarPainter.maxLidarDistance)
    }
}