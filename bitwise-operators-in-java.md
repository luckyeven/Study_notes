# Bitwise Operators in Java

Java int type is 32 bits long.

The left-most significant bit represents the sign bit

{% code lineNumbers="true" %}
```java
public class linkedlist {

    public static void print(int num) {
        for (int i = 31; i >= 0; i--) {
            System.out.print((num & (1 << i)) == 0 ? "0" : "1");

        }
    }

    public static void main(String[] args) {

        // int is 32 bit.
        int num = 2432423;
        print(num);

    }
}

```
{% endcode %}

Output:

```bash
00000000001001010001110110100111
```
