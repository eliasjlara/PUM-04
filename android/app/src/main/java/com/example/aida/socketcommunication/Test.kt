class TestArray(){
    fun emptyByteArray() : ByteArray {
        return ByteArray(0)
    }
}
fun main(args: Array<String>){
    val test = TestArray()
    val empty = test.emptyByteArray()
    println("Empty array size: ${empty.size}")
}