---
description: 'Caluculate 1! + 2! + 3! ... + N!: for a given N'
---

# sum k!

{% code lineNumbers="true" %}
```java
public static long f(int N) {
        long ans = 0;
        long cur = 1;
        for (int i = 1; i <= N; i++) {
            cur = cur * i;
            ans += cur;
        }

        return ans;
    }
```
{% endcode %}
