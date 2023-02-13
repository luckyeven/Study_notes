# Selection sort

<figure><img src="https://www.simplilearn.com/ice9/free_resources_article_thumb/Selection-Sort-Soni/what-is-selection-sort.png" alt=""><figcaption><p>Selection sort</p></figcaption></figure>

{% code lineNumbers="true" %}
```java
public static void selectSort(int[] arr){
        if (arr == null || arr.length < 2){
            return;
        }

        int N = arr.length;
        for(int i = 0; i < N; i++) {
            int minValueIndex = i;
            for (int j = i+1; j < N; j++){
                minValueIndex = arr[j] < arr[minValueIndex] ? j : minValueIndex;
            }
            swap(arr, i, minValueIndex);
        }

    }
    public static void swap(int [] arr, int left, int right){
        if(left == right){
            return;
        }

        arr[left] = arr[left] ^ arr[right];
        arr[right] = arr[left] ^ arr[right];
        arr[left] = arr[left] ^ arr[right];
    }
```
{% endcode %}
