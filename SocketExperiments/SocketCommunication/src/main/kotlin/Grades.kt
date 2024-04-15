package org.example

class Grades {
    fun getGrade(grade: Int): String {
        if (grade < 0 || grade > 100) {
            throw IllegalArgumentException("Grade must be between 0 and 100")
        } else if (grade >= 90) {
            return "A"
        } else if (grade >= 80) {
            return "B"
        } else if (grade >= 70) {
            return "C"
        } else if (grade >= 60) {
            return "D"
        } else {
            return "F"
        }
    }
}