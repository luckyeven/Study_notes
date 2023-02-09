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

## Alternatively

{% code lineNumbers="true" %}
```python
from django.http import HttpResponse 
def my_view(request): 
    # ... 
    if condition==True: 
        return HttpResponse('<h1>Page not found</h1>', status_code='404') 
    else: 
        return HttpResponse('<h1>Page was found</h1>') 
```
{% endcode %}

## raise Http404 exception

{% code lineNumbers="true" %}
```python
from django.http import Http404, HttpResponse 
from .models import Product 

def detail(request, id): 
    try: 
        p = Product.objects.get(pk=id) 
    except Product.DoesNotExist: 
        raise Http404("Product does not exist") 
    return HttpResponse("Product Found") 
```
{% endcode %}

## Django Debug model

{% code lineNumbers="true" %}
```python
# DEBUG is set to True in default
DEBUG = False

# Add possible host
ALLOWED_HOSTS =['*']
```
{% endcode %}
