import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.example.Grades

class GradesTest {

    @Test
    fun testGetGradeA() {
        val grades = Grades()
        assertEquals("A", grades.getGrade(95))
    }
    @Test
    fun testGetGradeB() {
        val grades = Grades()
        assertEquals("B", grades.getGrade(85))
    }
    @Test
    fun testGetGradeC() {
        val grades = Grades()
        assertEquals("C", grades.getGrade(75))
    }
    @Test
    fun testGetGradeD() {
        val grades = Grades()
        assertEquals("D", grades.getGrade(65))
    }
    @Test
    fun testGetGradeF() {
        val grades = Grades()
        assertEquals("F", grades.getGrade(55))
    }
    @Test
    fun testGetGradeInvalid() {
        val grades = Grades()
        assertThrows(IllegalArgumentException::class.java) {
            grades.getGrade(-1)
        }
        assertThrows(IllegalArgumentException::class.java) {
            grades.getGrade(101)
        }
    }
}