# Regular expression in URLs

## RegEx

RegEx are set of characters that specify a pattern and are used to search for or find patterns in a string. Programmer can use RegEx to perform:

* Extraction and validation
* Advanced searching
* Group searches
* Find and replace

Suppose we have a URL like this:\
`https://www.example.com/main/1`

{% code lineNumbers="true" %}
```python
# urls.py(app)

urlpatterns = [
    path('main/<int:id>', views.function),
]
```
{% endcode %}

Instead of using an angel bracket, Regular expressions can be used.

<pre class="language-python" data-line-numbers><code class="lang-python"># urls.py(app)
from django.urls import path, re_path
from . import views

<strong>urlpatterns = [
</strong><strong>    path('main/&#x3C;int:id>', views.function),
</strong><strong>    re_path(r'^main/([0-9]{2})/$', views.function, name='main'),
</strong><strong>]
</strong></code></pre>
