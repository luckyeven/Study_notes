# URL Namespacing and Views

### reverse() function

{% code lineNumbers="true" %}
```python
python3 manage.py shell
`
>>> from django.urls import reverse 
>>> reverse('index') 
'/demo/' 
```
{% endcode %}

## Application namespace

{% code lineNumbers="true" %}
```python
#demoapp/urls.py
from django.urls import path  
from . import views    
app_name='demoapp' 
urlpatterns = [  
    path('', views.index, name='index'),      
] 
```
{% endcode %}

The `app_name` defines the application namespace so that the views in this app are identified by it.

To obtain the URL path of the `index()` function, call the `reverse()` function by prepending the namespace to it.

```
>>> reverse('demoapp:index') 
'/demo/' 
```
