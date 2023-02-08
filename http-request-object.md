---
description: Get a better understanding of how http maps to the common CRUD operations
---

# HTTP Request Object

## views.py



In Django, a view is a function designed to handle a web request and return a web response such as an HTML document.

{% code lineNumbers="true" %}
```python
from django.http import HttpResponse

def home(request): # pass a request object
    content = "<html><body><h1>Welcome</h1></body></html>"
    return HTTPResponse(content) # return a HTML content
```
{% endcode %}

We can also perform other programming logic inside of view functions such as:

* Process data for email and forms
* Retrieve data from the database
* Transforming data
* Rendering templates

Only creating view function is not enough to make the request-response work, The view function needs to be mapped to a URL so when the request to the URL is made, the view function gets called. This process is known as `Routing.`

Inside of app folder, create a new file called urls.py.

### urls.py(app)

{% code lineNumbers="true" %}
```python
from django.urls import path
from . import views

urlpatterns = [
    path('', views.home),
    # first argument is the route, second argument is the view
    # which contains the relative path and the name of the view function
]
```
{% endcode %}

### urls.py(project)

When the user makes a URL request, this request is first handled by the urls.py at the project level. We can use the `include()` function in the `urls.py` file at the project level.

{% code lineNumbers="true" %}
```python
# urls.py(project)
from django.contrib import admin
from django.urls import path, include

urlpatterns = [
    path('admin/', admin.site.urls),
    paht('', include('<name of app>.urls'))
]
```
{% endcode %}

The project level `urls.py` can inherit the app level URL configurations.

### GET and POST methods

<pre class="language-python" data-line-numbers><code class="lang-python">
from django.shortcuts import render     

def myview(request): 
    if request.method=='GET': 
        val = request.GET['key'] 
        #perform read or delete operation on the model 
    if request.method=='POST': 
        val = request.POST['key'] 
        #perform insert or update operation on the model 
    
<strong>    return render(request, 'mytemplate.html', context)
</strong>
</code></pre>

Django offers a more concise alternative in the form of a class-based view.

{% code lineNumbers="true" %}
```python

from django.views import View 
class MyView(View): 
    def get(self, request): 
        # logic to process GET request
        return HttpResponse('response to GET request') 
 
    def post(self, request): 
        # <logic to process POST request> 
        return HttpResponse('response to POST request') 
```
{% endcode %}

