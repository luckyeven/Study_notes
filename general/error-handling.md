# Error Handling

{% code lineNumbers="true" %}
```python
# views.py(project)

from django.http import HttpResponse

def handler404(request, exception):
    return HttpResponse("404: Page not Found! ")

def home(request):
    return HttpResponse("Home page")
```
{% endcode %}
